/*! ----------------------------------------------------------------------------
 * @file    tdoa_solver.c
 * @brief   TDOA multilateration solver — Gauss-Newton iterative least squares
 *
 *          Uses hardware FPU (float) on Cortex-M4. Solves for tag position
 *          given anchor positions (cm) and TDOA measurements (dtu).
 *
 *          toa[0] is the reference anchor (beacon = anchor 0).
 *          toa[i] (i=1..N-1) are the slave blink TOAs.
 *          tdoa[i] = toa[i] - toa[0]  (in DW3000 device time units)
 *
 *          cm per DWT time unit = CM_PER_DTU_F (shared_defines.h)
 *            = 0.46907 cm/dtu
 *============================================================================*/
#include <math.h>
#include <shared_defines.h>
#include "downlink_protocol.h"

extern void test_run_info(unsigned char *data);

/* P2-9: the conversion constant is the one in shared_defines.h — the local
 * copy (SPEED_OF_LIGHT / 499.2 MHz / 128) had drifted from it by 0.006 %. */
#define CM_PER_DTU   CM_PER_DTU_F

#define MAX_ITER         12
#define CONV_THRESH      0.10f    /* convergence: step length < 0.10 cm */
#define CONV_DIFF_THRESH 0.02f    /* early exit: RMS improvement < 0.02 cm */
#define SING_THRESH      1e-6f    /* singular matrix threshold */
#define DAMP_FACTOR      0.3f     /* damping when residual increases */
#define DAMP_DIVERGING   0.15f   /* stronger damping when diverging */
#define OUTLIER_THRESH   3.0f     /* outlier = |residual| > OUTLIER_THRESH * rms */
#define MIN_WEIGHT_KEEP  0.1f     /* minimum weight to keep anchor in solve */

/* P2-10: when every anchor stands at the same height the vertical direction
 * carries no information at all: the 3D normal equations are singular in z
 * and the solver would only be fitting noise.  Below this spread the
 * network is treated as a horizontal plane and the solve is done in 2D. */
#define SAME_PLANE_TOL_CM  1.0f

/*============================================================================
 * solve_3x3 — Cramer's rule for H * x = g  (H is 3x3, g and x are 3x1)
 * Returns 0 on success, -1 if near-singular.
 *============================================================================*/
static int solve_3x3(float H[3][3], const float g[3], float x[3])
{
    float det;
    float a, b, c, d, e, f, gh, i;

    a = H[0][0]; b = H[0][1]; c = H[0][2];
    d = H[1][0]; e = H[1][1]; f = H[1][2];
    { float _gh = H[2][0]; gh = _gh; }
    { float _i  = H[2][1]; i  = _i; }
    /* H[2][2] used directly below */

    det = a * (e * H[2][2] - f * i)
        - b * (d * H[2][2] - f * gh)
        + c * (d * i - e * gh);

    if (fabsf(det) < SING_THRESH)
        return -1;

    /* Cramer's rule: x_k = det(H_k) / det(H) where H_k replaces column k with g */
    {
        float inv = 1.0f / det;
        float h22 = H[2][2];

        x[0] = (g[0] * (e * h22 - f * i)
              + b * (f * g[2] - g[1] * h22)
              + c * (g[1] * i - e * g[2])) * inv;

        x[1] = (a * (g[1] * h22 - f * g[2])
              + g[0] * (f * gh - d * h22)
              + c * (d * g[2] - g[1] * gh)) * inv;

        x[2] = (a * (e * g[2] - g[1] * i)
              + b * (g[1] * gh - d * g[2])
              + g[0] * (d * i - e * gh)) * inv;
    }

    return 0;
}

/*============================================================================
 * tdoa_solve — Gauss-Newton multilateration
 *
 *   anchors[]   : N anchor positions (id, x_cm, y_cm, z_cm)
 *   toa_dtu[]   : N TOA timestamps in DWT time units (toa_dtu[0] = beacon)
 *   num_anchors : total anchors (N ≥ 3 for 2D, N ≥ 4 for 3D)
 *   x/y/z_cm_out: output position in cm
 *   rms_out     : final RMS residual in cm
 *
 *   Returns 0 on success, -1 on singular matrix, -2 on too few anchors.
 *============================================================================*/
/*============================================================================
 * gn_solve — ONE Gauss-Newton run, started from a given seed
 *
 *   anchors[]   : N anchor positions (id, x_cm, y_cm, z_cm)
 *   tdoa_cm[]   : N-1 TDOA values in cm, relative to anchor 0
 *   use_2d      : solve in the horizontal plane only (z is not solved for)
 *   x0/y0/z0    : the seed of the iteration
 *
 *   Returns 0 on success, -1 on a singular matrix.
 *============================================================================*/
static int gn_solve(const dl_anchor_entry_t *anchors,
                    const float *tdoa_cm,
                    uint8_t num_anchors,
                    int use_2d,
                    float x0, float y0, float z0,
                    float *x_out, float *y_out, float *z_out,
                    float *rms_out)
{
    float r[DL_MAX_ANCHORS];
    float x = x0, y = y0, z = z0;
    int iter, k;
    uint8_t n = num_anchors;

    /* Outlier tracking: each measurement's weight (0 = rejected) */
    float weight[DL_MAX_ANCHORS];
    for (k = 0; k < DL_MAX_ANCHORS; k++) weight[k] = 1.0f;

    float prev_rms = 1e9f;
    int diverge_count = 0;

    /* 3. Gauss-Newton iteration with outlier rejection */
    for (iter = 0; iter < MAX_ITER; iter++)
    {
        float H[3][3], g[3];
        float res[DL_MAX_ANCHORS];
        float rms = 0, weighted_rms = 0;
        uint8_t used_count = 0;

        /* Init accumulators */
        H[0][0] = H[0][1] = H[0][2] = 0;
        H[1][0] = H[1][1] = H[1][2] = 0;
        H[2][0] = H[2][1] = H[2][2] = 0;
        g[0] = g[1] = g[2] = 0;

        /* 3a. Compute ranges */
        for (k = 0; k < (int)n; k++)
        {
            float dx = x - (float)anchors[k].x_cm;
            float dy = y - (float)anchors[k].y_cm;
            float dz = z - (float)anchors[k].z_cm;
            r[k] = sqrtf(dx * dx + dy * dy + dz * dz);
            if (r[k] < 1.0f) r[k] = 1.0f;
        }

        /* 3b/3c. Compute residuals + accumulate Jacobian (skip weight=0) */
        for (k = 1; k < (int)n; k++)
        {
            if (weight[k] < 0.5f) continue;

            float pred = r[k] - r[0];
            res[k] = pred - tdoa_cm[k];
            float w = weight[k];
            rms += res[k] * res[k];
            weighted_rms += (res[k] * res[k]) * w;
            used_count++;

            /* Jacobian row for anchor k relative to anchor 0 */
            float dx0 = (x - (float)anchors[0].x_cm) / r[0];
            float dy0 = (y - (float)anchors[0].y_cm) / r[0];
            float dz0 = (z - (float)anchors[0].z_cm) / r[0];
            float dxk = (x - (float)anchors[k].x_cm) / r[k];
            float dyk = (y - (float)anchors[k].y_cm) / r[k];
            float dzk = (z - (float)anchors[k].z_cm) / r[k];

            float J0 = dxk - dx0;
            float J1 = dyk - dy0;
            float J2 = dzk - dz0;

            H[0][0] += J0 * J0 * w;
            H[0][1] += J0 * J1 * w;
            H[0][2] += J0 * J2 * w;
            H[1][1] += J1 * J1 * w;
            H[1][2] += J1 * J2 * w;
            H[2][2] += J2 * J2 * w;
            g[0] -= J0 * res[k] * w;
            g[1] -= J1 * res[k] * w;
            g[2] -= J2 * res[k] * w;
        }

        /* Fill lower triangle */
        H[1][0] = H[0][1];
        H[2][0] = H[0][2];
        H[2][1] = H[1][2];

        float denom = (float)(used_count > 0 ? used_count - 1 : 1);
        rms = sqrtf(rms / denom);
        float w_rms = sqrtf(weighted_rms / denom);

        /* 3d. Outlier rejection: update weights based on residuals */
        if (iter > 0 && used_count >= 3)
        {
            for (k = 1; k < (int)n; k++)
            {
                if (weight[k] < 0.5f) continue;
                float thresh = OUTLIER_THRESH * w_rms;
                if (thresh < 1.0f) thresh = 1.0f;
                if (fabsf(res[k]) > thresh)
                {
                    weight[k] *= 0.5f;
                    if (weight[k] < MIN_WEIGHT_KEEP) weight[k] = 0.0f;
                }
            }
        }

        /* 3e. Adaptive damping: track diverging iterations */
        float damp = DAMP_FACTOR;
        if (rms > prev_rms && iter > 0)
        {
            diverge_count++;
            if (diverge_count >= 2) damp = DAMP_DIVERGING;
        }
        else
        {
            diverge_count = 0;
        }

        /* 3f. Early exit: RMS improvement below threshold */
        if (iter > 0 && fabsf(prev_rms - w_rms) < CONV_DIFF_THRESH)
            break;

        /* 3g. Solve */
        {
            float H2[3][3], g2[3];
            float det;
            uint8_t r1, c1;

            if (use_2d)
            {
                for (r1 = 0; r1 < 2; r1++)
                    for (c1 = 0; c1 < 2; c1++)
                        H2[r1][c1] = H[r1][c1];
                g2[0] = g[0]; g2[1] = g[1];

                det = H2[0][0] * H2[1][1] - H2[0][1] * H2[1][0];
                if (fabsf(det) < SING_THRESH)
                {
                    H2[0][0] += 1e-3f; H2[1][1] += 1e-3f;
                    det = H2[0][0] * H2[1][1] - H2[0][1] * H2[1][0];
                }

                float inv = 1.0f / det;
                float dx = (g2[0] * H2[1][1] - H2[0][1] * g2[1]) * inv;
                float dy = (H2[0][0] * g2[1] - g2[0] * H2[1][0]) * inv;

                dx *= damp;
                dy *= damp;

                x += dx;
                y += dy;

                if (sqrtf(dx * dx + dy * dy) < CONV_THRESH)
                    break;
            }
            else
            {
                for (r1 = 0; r1 < 3; r1++)
                    for (c1 = 0; c1 < 3; c1++)
                        H2[r1][c1] = H[r1][c1];
                g2[0] = g[0]; g2[1] = g[1]; g2[2] = g[2];

                float delta[3];
                if (solve_3x3(H2, g2, delta) != 0)
                    return -1;

                delta[0] *= damp;
                delta[1] *= damp;
                delta[2] *= damp;

                x += delta[0];
                y += delta[1];
                z += delta[2];

                if (sqrtf(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]) < CONV_THRESH)
                    break;
            }
        }

        prev_rms = w_rms;
    }

    *x_out = x;
    *y_out = y;
    *z_out = z;
    *rms_out  = prev_rms;
    return 0;
}

/*============================================================================
 * tdoa_solve — multi-start Gauss-Newton
 *
 *   anchors[]   : N anchor positions (id, x_cm, y_cm, z_cm)
 *   toa_dtu[]   : N TOA timestamps in DWT time units (toa_dtu[0] = beacon)
 *   num_anchors : total anchors (N ≥ 3 for 2D, N ≥ 4 for 3D)
 *   x/y/z_cm_out: output position in cm
 *   rms_out     : final RMS residual in cm
 *
 *   Returns 0 on success, -1 on singular matrix, -2 on too few anchors.
 *
 *   P2-11: the seed decides whether the iteration finds the right minimum.
 *   The tag moves a few centimetres between two rounds, so the PREVIOUS
 *   frame's solution is a far better seed than the centroid of the anchors
 *   — and the centroid is kept as a second start, so that one bad previous
 *   solution cannot trap the iteration in a local minimum for ever.  The
 *   start with the smaller residual wins.
 *============================================================================*/

/* The last solution, used as the seed of the next solve. */
static float s_prev_x_cm = 0.0f;
static float s_prev_y_cm = 0.0f;
static float s_prev_z_cm = 0.0f;
static int   s_prev_valid = 0;

int tdoa_solve(const dl_anchor_entry_t *anchors,
               const uint64_t *toa_dtu,
               uint8_t num_anchors,
               float *x_cm_out, float *y_cm_out, float *z_cm_out,
               float *rms_out)
{
    float tdoa_cm[DL_MAX_ANCHORS];
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
    int k;
    uint8_t n = num_anchors;
    int use_2d = 0;

    if (n < 2)
        return -2;

    /* 1. Convert TDOA to cm */
    {
        uint64_t ref = toa_dtu[0];
        for (k = 1; k < (int)n; k++)
        {
            int64_t diff = (int64_t)(toa_dtu[k] - ref);
            tdoa_cm[k] = (float)diff * CM_PER_DTU;
        }
    }

    /* 2. Centroid of the anchors — the neutral seed */
    {
        float sx = 0, sy = 0, sz = 0;
        for (k = 0; k < (int)n; k++)
        {
            sx += (float)anchors[k].x_cm;
            sy += (float)anchors[k].y_cm;
            sz += (float)anchors[k].z_cm;
        }
        cx = sx / (float)n;
        cy = sy / (float)n;
        cz = sz / (float)n;
    }

    /* 3. 2D or 3D?
     *    P2-10: also 2D when every anchor stands at the same height — the
     *    vertical direction is then unobservable, the 3D normal equations
     *    are singular in z and the solver would only be fitting noise.
     *
     *    A 3D solve needs one measurement MORE than the three unknowns:
     *    with exactly four anchors the system is exactly determined, so a
     *    noisy TDOA is fitted exactly — the residual stays near zero while
     *    the position is off by a metre (measured on the host simulation:
     *    5 cm of TDOA noise → 94 cm of position error).  Five anchors make
     *    it over-determined and the least-squares fit stable. */
    if (n < 5)
        use_2d = 1;
    {
        float zmin = (float)anchors[0].z_cm;
        float zmax = zmin;
        for (k = 1; k < (int)n; k++)
        {
            float az = (float)anchors[k].z_cm;
            if (az < zmin) zmin = az;
            if (az > zmax) zmax = az;
        }
        if (zmax - zmin <= SAME_PLANE_TOL_CM)
            use_2d = 1;
    }

    /* 4. Multi-start: the previous solution and the centroid.
     *
     *    When the 3D system turns out to be singular the whole solve is
     *    repeated in the horizontal plane.  That is not a rare corner:
     *    with a tag INSIDE the polygon of the anchors the rows of the
     *    TDOA Jacobian are nearly parallel, the normal matrix is
     *    ill-conditioned, and the old code returned -1 — the tag printed
     *    NO position at all for a four-anchor network. */
    {
        float bx = 0.0f, by = 0.0f, bz = 0.0f, br = 0.0f;
        int have = 0;
        int used_2d = use_2d;
        int attempts = use_2d ? 1 : 2;
        int attempt;

        for (attempt = 0; attempt < attempts && !have; attempt++)
        {
            int try_2d = use_2d || (attempt > 0);

            if (s_prev_valid)
            {
                float tx, ty, tz, tr;
                if (gn_solve(anchors, tdoa_cm, n, try_2d,
                             s_prev_x_cm, s_prev_y_cm, s_prev_z_cm,
                             &tx, &ty, &tz, &tr) == 0)
                {
                    bx = tx; by = ty; bz = tz; br = tr; have = 1; used_2d = try_2d;
                }
            }
            {
                float tx, ty, tz, tr;
                if (gn_solve(anchors, tdoa_cm, n, try_2d, cx, cy, cz,
                             &tx, &ty, &tz, &tr) == 0)
                {
                    if (!have || tr < br) { bx = tx; by = ty; bz = tz; br = tr; have = 1; used_2d = try_2d; }
                }
            }
        }
        if (!have)
            return -1;

        x = bx; y = by; z = bz;

        /* Remember the solution as the seed of the next round. */
        s_prev_x_cm = bx; s_prev_y_cm = by; s_prev_z_cm = bz; s_prev_valid = 1;

        /* In the horizontal-plane solve the height carries no information:
         * report the plane of the anchors, not the seed. */
        if (used_2d) z = cz;

        *rms_out = br;
    }

    *x_cm_out = x;
    *y_cm_out = y;
    *z_cm_out = z;
    return 0;
}
