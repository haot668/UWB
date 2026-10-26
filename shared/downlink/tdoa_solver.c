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
int tdoa_solve(const dl_anchor_entry_t *anchors,
               const uint64_t *toa_dtu,
               uint8_t num_anchors,
               float *x_cm_out, float *y_cm_out, float *z_cm_out,
               float *rms_out)
{
    float tdoa_cm[DL_MAX_ANCHORS];
    float r[DL_MAX_ANCHORS];
    float x, y, z;
    int iter, k;
    uint8_t n;
    int use_2d = 0;

    /* Outlier tracking: each measurement's weight (0 = rejected) */
    float weight[DL_MAX_ANCHORS];
    for (k = 0; k < DL_MAX_ANCHORS; k++) weight[k] = 1.0f;

    n = num_anchors;
    if (n < 2)
        return -2;

    /* Use 2D mode (fix z) when fewer than 4 anchors */
    if (n < 4)
        use_2d = 1;

    /* 1. Convert TDOA to cm */
    {
        uint64_t ref = toa_dtu[0];
        for (k = 1; k < (int)n; k++)
        {
            int64_t diff = (int64_t)(toa_dtu[k] - ref);
            tdoa_cm[k] = (float)diff * CM_PER_DTU;
        }
    }

    /* 2. Initial guess: centroid of all anchors */
    {
        float sx = 0, sy = 0, sz = 0;
        for (k = 0; k < (int)n; k++)
        {
            sx += (float)anchors[k].x_cm;
            sy += (float)anchors[k].y_cm;
            sz += (float)anchors[k].z_cm;
        }
        x = sx / (float)n;
        y = sy / (float)n;
        z = sz / (float)n;
    }

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

    *x_cm_out = x;
    *y_cm_out = y;
    *z_cm_out = z;
    *rms_out  = prev_rms;
    return 0;
}
