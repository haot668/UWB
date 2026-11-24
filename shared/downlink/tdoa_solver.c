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
 *
 *          ON THE "THE POSITION IS ALWAYS THE CENTROID" SYMPTOM
 *          ----------------------------------------------------
 *          The centroid of the anchors is one of the two seeds of the
 *          multi-start, so every failure of the iteration leaves the
 *          answer sitting exactly on it — and a fit that never moved has
 *          a small residual, so the quality score (which is computed
 *          from the residual) rated it as a GOOD fix.  The guards in this
 *          file (too few anchors, coincident anchors, a singular or badly
 *          conditioned normal matrix, a residual outside the physical
 *          range of the geometry) turn those failures into a REFUSED
 *          solve instead of a position.
 *
 *          What is NOT fixable here: if the measurements themselves carry
 *          no information (every TDOA difference is zero), the only fit is
 *          the equidistant point — which for a symmetric layout IS the
 *          centroid.  That input is indistinguishable from the perfectly
 *          legitimate case of a tag that really does stand at the
 *          centroid, so no test inside the solver can tell them apart.
 *          The discrimination has to be made where the provenance of the
 *          measurement is known — in the tag's TDOA assembly.
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
 * The three guards below all serve one rule: 宁可不出，也不输出质心.
 *
 * "Never publish a position you cannot justify" — in particular never
 * publish the anchor centroid.  The centroid is one of the two seeds of
 * the multi-start, so ANY failure of the iteration leaves the answer
 * sitting exactly on it, and the residual of a degenerate fit is small
 * (a fit that never moved has no residual) — which means the quality
 * score, which is computed from the residual, happily rated it as a
 * GOOD fix.  That is how "the position is always the centroid" can
 * happen with a healthy-looking quality number.
 *
 * The three ways it can happen are closed here:
 *
 *   1. too few anchors        -> underdetermined, the step is arbitrary
 *   2. coincident anchors     -> duplicate equations, same as (1)
 *   3. a singular normal matrix -> the step vanishes, the answer stays
 *                                   on the seed
 *
 * All three now make the solve FAIL (-1) instead of returning a number.
 *==========================================================================*/

/* Two anchors closer than this to each other stand at the same spot: they
 * produce the same TDOA equation, so they add no information.  10 cm is far
 * below any sane anchor spacing (UWB anchors are metres apart) and far
 * above the 1 cm quantisation of the coordinate field. */
#define COINCIDENT_CM     10.0f

/* The smallest eigenvalue's share of the trace of the horizontal normal
 * matrix.  It measures how much of the fit the WEAKEST direction carries:
 * at a share of 0.01 the worst direction is amplified 100x over the best
 * one, i.e. the noise in that direction is multiplied by ten.
 *
 * Measured on the host test (test/solver_host_test.c), at the true tag
 * position:
 *   four anchors on a square, tag inside    -> 0.200  (well conditioned)
 *   four anchors on a square, tag at centre  -> 0.250
 *   six anchors in a ring, tag inside        -> 0.217
 *   three anchors in a triangle              -> 0.142
 *   four anchors in a LINE, tag off the line -> 0.020  (still usable: a
 *        corridor locates the tag across the line, and the fit was 4 cm
 *        off — the weak direction is the along-the-corridor one)
 *   four anchors in a LINE, tag ON the line  -> 0.000  (the two equations
 *        are parallel there: the fit was 65 cm off while claiming a 2.8 cm
 *        residual, i.e. a confident wrong answer)
 * The threshold sits between the last two. */
#define COND_MIN_RATIO    0.010f

/* P2-10: when every anchor stands at the same height the vertical direction
 * carries no information at all: the 3D normal equations are singular in z
 * and the solver would only be fitting noise.
 *
 * The old test was a HEIGHT SPREAD ("are all the anchors within 1 cm of the
 * same height?").  That test was meaningless: 1 cm of spread still leaves
 * the z column of the TDOA Jacobian almost empty, so the height error is
 * amplified by the ratio of the horizontal extent to the height spread --
 * with 1 cm of spread over 20 m that is a factor of 2000, and a 5 cm
 * residual would put the height kilometres away.  A spread threshold cannot
 * express that.
 *
 * The test used now is the GEOMETRIC ONE: how much of the information in
 * the normal matrix sits in the vertical direction.  z_info is the share
 * of the trace of H that belongs to the z column,
 *
 *      z_info = H[2][2] / (H[0][0] + H[1][1] + H[2][2])
 *
 * With coplanar anchors the z column collapses and z_info goes to zero.
 * With a metre of height spread over 20 m of floor it is still tiny, and
 * the height error is amplified by ~20 -- a 5 cm residual becomes 1 m of
 * height, which is useless.
 *
 * The threshold is the share below which the height stops being worth
 * anything: at z_info = 0.02 the vertical direction carries 1/50 of the
 * information, so the height error is ~50x the residual -- 5 cm of
 * residual, 2.5 m of height.  Below the threshold the network is solved
 * in the horizontal plane and the height is reported as NOT VALID, so the
 * caller can tell "the height is the anchor plane" from "the height was
 * measured". */
#define Z_INFO_MIN  0.02f

/*============================================================================
 * z_result_usable — may this 3D result be used?
 *
 * A 3D fit is only worth keeping when the vertical direction carries
 * enough information to be worth the error amplification it causes.  The
 * share of the normal matrix that sits in z is the measure of that; a
 * result below the threshold is discarded and the horizontal solve takes
 * over (the multi-start already has that attempt lined up).
 *==========================================================================*/
static int z_result_usable(int try_2d, float z_info)
{
    if (try_2d) return 1;
    return (z_info >= Z_INFO_MIN) ? 1 : 0;
}

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
 * gn_solve — ONE Gauss-Newton run, started from a given seed
 *
 *   anchors[]   : N anchor positions (id, x_cm, y_cm, z_cm)
 *   tdoa_cm[]   : N-1 TDOA values in cm, relative to anchor 0
 *   use_2d      : solve in the horizontal plane only (z is not solved for)
 *   x0/y0/z0    : the seed of the iteration
 *   n_used_out  : number of surviving TDOA measurements (may be NULL)
 *   z_info_out  : the share of the normal matrix that belongs to the
 *                 vertical direction, i.e. how much the height was
 *                 actually determined by the measurements (may be NULL)
 *
 *   Returns 0 on success, -1 on a singular matrix.
 *============================================================================*/
static int gn_solve(const dl_anchor_entry_t *anchors,
                    const float *tdoa_cm,
                    uint8_t num_anchors,
                    int use_2d,
                    float x0, float y0, float z0,
                    float *x_out, float *y_out, float *z_out,
                    float *rms_out,
                    uint8_t *n_used_out, float *z_info_out)
{
    float r[DL_MAX_ANCHORS];
    float x = x0, y = y0, z = z0;
    int iter, k;
    uint8_t n = num_anchors;
    /* The state of the LAST iteration: how much information the vertical
     * direction carried. */
    float  last_z_info = 0.0f;

    /* Outlier tracking: each measurement's weight (0 = rejected) */
    float weight[DL_MAX_ANCHORS];
    for (k = 0; k < DL_MAX_ANCHORS; k++) weight[k] = 1.0f;

    float prev_rms = 1e9f;
    int diverge_count = 0;
    /* Was the last POSITION UPDATE made with a singular normal matrix? */
    int last_step_singular = 0;
    /* The conditioning of the horizontal normal matrix of the last
     * iteration (see COND_MIN_RATIO). */
    float cond_ratio = 1.0f;
    /* The horizontal normal matrix of the last iteration. */
    float Hlast[3][3];

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

        /* Keep the last normal matrix: its conditioning is judged after the
         * loop, when it is known which step produced the final position. */
        {
            int rr, cc;
            for (rr = 0; rr < 3; rr++)
                for (cc = 0; cc < 3; cc++)
                    Hlast[rr][cc] = H[rr][cc];
        }

        /* How much of the normal matrix belongs to the vertical direction,
         * and how many anchors are still in the fit.  See Z_INFO_MIN. */
        {
            float tr = H[0][0] + H[1][1] + H[2][2];
            last_z_info = (tr > 0.0f) ? (H[2][2] / tr) : 0.0f;
        }

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
                    /* The normal matrix is singular: the measurements do not
                     * determine the position in one direction (the classic
                     * case is the tag standing on the line through the
                     * anchors, where the two TDOA equations have parallel
                     * gradients).
                     *
                     * The OLD code added 1e-3 to the diagonal and carried on.
                     * That turns the step into a tiny nudge, so the solution
                     * stays wherever the seed was — and one of the seeds is
                     * the anchor centroid.  The fit was then published as a
                     * position, with a small residual and a good quality
                     * score, while being up to a metre off.  The iteration
                     * is still kept alive below (a bad step can be
                     * recovered from), but the singularity is remembered and
                     * the run is FAILED if the last step still used one. */
                    last_step_singular = 1;
                    H2[0][0] += 1e-3f; H2[1][1] += 1e-3f;
                    det = H2[0][0] * H2[1][1] - H2[0][1] * H2[1][0];
                }
                else
                {
                    last_step_singular = 0;
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

    /* ── The verdict on the geometry ──────────────────────────────────────
     *
     * Two things are checked here, and both of them end in the solve
     * FAILING rather than in a number being returned:
     *
     * 1. The last position update used a SINGULAR normal matrix.  That is
     *    the "the iteration had nothing to work with" case: the answer is
     *    the seed, and the seed is the anchor centroid.
     *
     * 2. The horizontal normal matrix is badly CONDITIONED: the weakest
     *    direction carries less than COND_MIN_RATIO of the information, so
     *    the noise in that direction is amplified far beyond what a
     *    position is worth.  The fit stays plausible-looking (a small
     *    residual) while being half a metre off. */
    {
        float tr  = Hlast[0][0] + Hlast[1][1];
        float det = Hlast[0][0] * Hlast[1][1] - Hlast[0][1] * Hlast[1][0];
        float disc = tr * tr - 4.0f * det;
        if (disc < 0.0f) disc = 0.0f;
        {
            float lmin = 0.5f * (tr - sqrtf(disc));
            cond_ratio = (tr > 1e-12f) ? (lmin / tr) : 0.0f;
        }
    }
    if (last_step_singular)
        return -1;
    if (cond_ratio < COND_MIN_RATIO)
        return -1;

    *x_out = x;
    *y_out = y;
    *z_out = z;
    *rms_out  = prev_rms;
    /* The count is the number of INLIER TDOA measurements (anchor k against
     * the reference anchor 0).  The reference itself produces no
     * measurement, so it is not counted -- the caller can compare this
     * against (num_anchors - 1) to get the inlier ratio.
     *
     * An anchor whose residual blew through the outlier threshold keeps a
     * weight of 0.5: it still contributes to the fit, but it is NOT an
     * inlier.  Counting it as one made the host's inlier ratio read 100%
     * while a measurement had just been flagged as an outlier, so the
     * quality score was optimistic exactly when it should not have been. */
    {
        uint8_t clean = 0;
        for (k = 1; k < (int)n; k++) {
            if (weight[k] >= 0.999f) clean++;
        }
        if (n_used_out)  *n_used_out  = clean;
    }
    if (z_info_out)  *z_info_out  = last_z_info;
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
 *   n_inlier_out: number of surviving TDOA measurements (may be NULL)
 *
 *   Returns  1 on success AND the height carries information,
 *            0 on success but the height carries NO information
 *              (coplanar anchors, or fewer than 5 anchors for a 3D fit),
 *           -1 on a geometry that cannot support a position (singular or
 *              badly conditioned normal matrix, coincident anchors, or a
 *              residual outside the physical range of the geometry),
 *           -2 on too few anchors.
 *
 *   The caller MUST NOT read the height as a measured one when the return
 *   value is 0 -- in that case the height is the plane of the anchors and
 *   is only a placeholder.
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
               float *rms_out,
               uint8_t *n_inlier_out)
{
    float tdoa_cm[DL_MAX_ANCHORS];
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
    int k, j;
    uint8_t n = num_anchors;
    int use_2d = 0;
    int z_valid = 0;
    /* The largest distance between two anchors: no TDOA can exceed it. */
    float spread = 0.0f;

    /* Two anchors make ONE TDOA, and one TDOA cannot place a tag in a
     * plane.  The old guard was "n < 2", which let a two-anchor call
     * through: the system was underdetermined, the normal matrix was
     * singular, and the answer came back as a position with an infinite
     * residual.  (The tag never made such a call — it checks n_valid < 3
     * itself — but the solver is a public API and must not hand out a
     * position it cannot justify.) */
    if (n < 3)
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

    /* 2b. The geometry has to be able to carry a position.
     *
     * Two anchors that stand at the same spot produce the SAME TDOA
     * equation, so they carry no information that the first one did not
     * already carry.  A set of four anchors of which three stand together
     * is therefore a TWO-anchor system wearing a four-anchor hat: the
     * normal matrix is singular, the step vanishes, and the answer stays
     * on the seed — the anchor centroid.  The old code published that.
     *
     * So the number of DISTINCT positions is what counts, not the number
     * of anchors. */
    {
        int n_distinct = 0;
        for (k = 0; k < (int)n; k++)
        {
            int dup = 0;
            for (j = 0; j < k; j++)
            {
                float dx = (float)anchors[k].x_cm - (float)anchors[j].x_cm;
                float dy = (float)anchors[k].y_cm - (float)anchors[j].y_cm;
                float dz = (float)anchors[k].z_cm - (float)anchors[j].z_cm;
                if (dx * dx + dy * dy + dz * dz
                    < COINCIDENT_CM * COINCIDENT_CM) { dup = 1; break; }
            }
            if (!dup) n_distinct++;
        }
        /* Two independent TDOAs are the minimum for a horizontal fix. */
        if (n_distinct < 3)
            return -1;
        /* A 3D fit needs three independent equations, i.e. four distinct
         * anchors.  With fewer, the horizontal solve is the only one that
         * can work. */
        if (n_distinct < 4)
            use_2d = 1;
    }

    /* 2c. The largest distance between two anchors.  It is the physical
     * bound of every TDOA, so it is also the bound of a meaningful
     * residual. */
    for (k = 0; k < (int)n; k++)
    {
        for (j = k + 1; j < (int)n; j++)
        {
            float dx = (float)anchors[k].x_cm - (float)anchors[j].x_cm;
            float dy = (float)anchors[k].y_cm - (float)anchors[j].y_cm;
            float dz = (float)anchors[k].z_cm - (float)anchors[j].z_cm;
            float d = sqrtf(dx * dx + dy * dy + dz * dz);
            if (d > spread) spread = d;
        }
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
    /* The vertical direction is NOT tested here any more: a 3D attempt is
     * made and its result is judged on the information the vertical
     * direction actually carried (Z_INFO_MIN, z_result_usable).  A spread
     * test cannot express that -- see the comment on Z_INFO_MIN. */

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
        uint8_t bn = 0;

        for (attempt = 0; attempt < attempts && !have; attempt++)
        {
            int try_2d = use_2d || (attempt > 0);

            if (s_prev_valid)
            {
                float tx, ty, tz, tr;
                uint8_t tu; float tzi;
                /* In the horizontal solve the height is FIXED at the anchor
                 * plane, so it is seeded with cz and not with the previous
                 * height: the residuals and the reported height then agree.
                 * (Before this the solve ran with the seed's height and
                 * only the REPORTED height was replaced afterwards, so the
                 * position that came out did not satisfy the height that
                 * was published with it.) */
                float sz = try_2d ? cz : s_prev_z_cm;
                if (gn_solve(anchors, tdoa_cm, n, try_2d,
                             s_prev_x_cm, s_prev_y_cm, sz,
                             &tx, &ty, &tz, &tr, &tu, &tzi) == 0
                    && z_result_usable(try_2d, tzi))
                {
                    bx = tx; by = ty; bz = tz; br = tr; bn = tu;
                    have = 1; used_2d = try_2d;
                }
            }
            {
                float tx, ty, tz, tr;
                uint8_t tu; float tzi;
                if (gn_solve(anchors, tdoa_cm, n, try_2d, cx, cy, cz,
                             &tx, &ty, &tz, &tr, &tu, &tzi) == 0
                    && z_result_usable(try_2d, tzi))
                {
                    if (!have || tr < br)
                    {
                        bx = tx; by = ty; bz = tz; br = tr; bn = tu;
                        have = 1; used_2d = try_2d;
                    }
                }
            }
        }
        if (!have)
            return -1;

        /* The residual has to be a number, and it has to be a plausible
         * one: no TDOA between two anchors can be larger than the
         * distance between them, so a residual beyond that spread is not
         * a fit to this network but the aftermath of a failed one.  (The
         * degenerate runs produced a residual of 1e9 cm — an infinite
         * one — and it was published as a position with a good-looking
         * quality score.) */
        if (!(br >= 0.0f) || !(br <= spread))
            return -1;

        /* The height is meaningful only if the solve was a 3D one. */
        z_valid = used_2d ? 0 : 1;

        x = bx; y = by; z = bz;

        /* Remember the solution as the seed of the next round. */
        s_prev_x_cm = bx; s_prev_y_cm = by; s_prev_z_cm = bz; s_prev_valid = 1;

        /* In the horizontal-plane solve the height carries no information:
         * the plane of the anchors.  The solve ran with that very height
         * as its seed, so the published x/y really do belong to it. */
        if (used_2d) z = cz;

        *rms_out = br;
        if (n_inlier_out) *n_inlier_out = bn;
    }

    *x_cm_out = x;
    *y_cm_out = y;
    *z_cm_out = z;
    /* 1 = solved and the height is meaningful, 0 = solved but the height
     * carries no information (coplanar anchors, or too few anchors for a
     * 3D fit).  The caller must NOT read a height in the second case as
     * a measured one. */
    return z_valid ? 1 : 0;
}
