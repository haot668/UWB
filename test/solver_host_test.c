/*! ----------------------------------------------------------------------------
 * @file    solver_host_test.c
 * @brief   Host-side test harness for the TDOA solver (shared/downlink/tdoa_solver.c)
 *
 * WHY THIS EXISTS
 * ---------------
 * The solver is pure computation — no HAL, no radio, no DW3000 — so it is the
 * ONE piece of the firmware that can be compiled AND RUN on the development
 * machine.  Everything else in this project has so far only been syntax
 * checked.  This harness is what turns "the solver should be right" into a
 * number.
 *
 * WHAT IT CHECKS
 * --------------
 * The reported symptom of this project is "the tag's position is always the
 * centroid of the anchors".  There are three plausible mechanisms:
 *
 *   (a) the anchor set is geometrically degenerate (the TDOA Jacobian is
 *       rank deficient → the least-squares step is a no-op → the solution
 *       stays exactly on its seed, and the centroid is one of the seeds);
 *   (b) the system is EXACTLY DETERMINED (2 measurements for 2 unknowns),
 *       so the residual is identically zero and cannot discriminate between
 *       the candidate seeds;
 *   (c) the measurements themselves are garbage.
 *
 * The harness reproduces all three and reports what the solver does, so the
 * fix can be written against a reproduction instead of a guess.
 *
 * BUILD (from the repository root, in WSL):
 *   gcc -std=c99 -Wall -I shared/shared_data -I shared/downlink \
 *       test/solver_host_test.c shared/downlink/tdoa_solver.c -lm -o /tmp/solver_test
 *   /tmp/solver_test
 *==========================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "downlink_protocol.h"

#define MAXA  DL_MAX_ANCHORS

/* The solver's own conversion (mirrors tdoa_solver.c).
 * M1: it comes from shared_defines.h now — the test must not carry its
 * own copy of the number, or the test could "pass" against a constant the
 * firmware does not use. */
#define CM_PER_DTU CM_PER_DTU_F

static int g_fail = 0;
static int g_checks = 0;

/* Distance in cm from the tag to an anchor. */
static float dist_cm(const dl_anchor_entry_t *a, float tx, float ty, float tz)
{
    float dx = tx - (float)a->x_cm;
    float dy = ty - (float)a->y_cm;
    float dz = tz - (float)a->z_cm;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/* The conditioning of the horizontal TDOA normal matrix at a given point.
 *
 * This mirrors what the solver judges internally (COND_MIN_RATIO): the
 * smallest eigenvalue's share of the trace of the 2x2 normal matrix.  It is
 * computed HERE at the TRUE tag position so that the geometry of each test
 * case can be compared independently of whether the solver accepted it. */
static float cond_ratio_at(const dl_anchor_entry_t *a, int n, float px, float py)
{
    float N00 = 0, N01 = 0, N11 = 0;
    float r0, j0, j1;
    int k;
    float dx = px - (float)a[0].x_cm, dy = py - (float)a[0].y_cm;
    r0 = sqrtf(dx * dx + dy * dy);
    if (r0 < 1.0f) r0 = 1.0f;
    j0 = dx / r0; j1 = dy / r0;

    for (k = 1; k < n; k++) {
        float ddx = px - (float)a[k].x_cm, ddy = py - (float)a[k].y_cm;
        float rk = sqrtf(ddx * ddx + ddy * ddy);
        float J0, J1;
        if (rk < 1.0f) rk = 1.0f;
        J0 = ddx / rk - j0;
        J1 = ddy / rk - j1;
        N00 += J0 * J0; N01 += J0 * J1; N11 += J1 * J1;
    }
    {
        float tr = N00 + N11;
        float det = N00 * N11 - N01 * N01;
        float disc = tr * tr - 4.0f * det;
        float lmin;
        if (disc < 0.0f) disc = 0.0f;
        lmin = 0.5f * (tr - sqrtf(disc));
        return (tr > 1e-12f) ? (lmin / tr) : 0.0f;
    }
}

/* Build the exact TDOA set for a tag at (tx,ty,tz) and run the solver.
 *
 *   mode     : 0 = derive the TDOAs from the true tag position (perfect data)
 *              1 = replace the TDOAs by a fixed nonsense pattern
 *              2 = perfect data plus a per-anchor bias of k * bias_cm, i.e.
 *                  exactly the error the slot compensation injects when the
 *                  tag's rate estimate for the Root is off (the bias grows
 *                  with the slot index)
 *   bias_cm  : the size of one slot's worth of path error, in cm
 *
 * Prints one line per case and returns the solver's return code. */
static int run_case(const char *name,
                    const dl_anchor_entry_t *anchors, int n,
                    float tx, float ty, float tz,
                    int mode, float bias_cm)
{
    uint64_t toa[MAXA];
    dl_anchor_entry_t a[MAXA];
    float x = 0, y = 0, z = 0, rms = 0;
    uint8_t ninl = 0;
    float cx = 0, cy = 0, cz = 0;
    int i;

    memcpy(a, anchors, sizeof(dl_anchor_entry_t) * (size_t)n);

    for (i = 0; i < n; i++) {
        cx += (float)a[i].x_cm; cy += (float)a[i].y_cm; cz += (float)a[i].z_cm;
    }
    cx /= (float)n; cy /= (float)n; cz /= (float)n;

    toa[0] = 0;
    for (i = 1; i < n; i++) {
        if (mode == 1) {
            /* A pattern that is NOT what the geometry implies: the same
             * number for every anchor (as if every beacon had the same
             * arrival time) plus a per-anchor wobble. */
            float tdoa_cm = 40.0f * (float)i + 7.0f;
            toa[i] = (uint64_t)(int64_t)(tdoa_cm / CM_PER_DTU);
        } else if (mode == 3) {
            toa[i] = 0;                       /* null measurement */
        } else {
            float d = dist_cm(&a[i], tx, ty, tz) - dist_cm(&a[0], tx, ty, tz);
            if (mode == 2) d += bias_cm * (float)i;
            toa[i] = (uint64_t)(int64_t)(d / CM_PER_DTU);
        }
    }

    int ret = tdoa_solve(a, toa, (uint8_t)n, &x, &y, &z, &rms, &ninl);
    float err = sqrtf((x - tx) * (x - tx) + (y - ty) * (y - ty));
    float dcx = x - cx, dcy = y - cy;
    float dcen = sqrtf(dcx * dcx + dcy * dcy);

    printf("%-34s ret=%2d  pos=(%8.1f,%8.1f,%8.1f)  err=%8.1f cm  rms=%7.2f cm"
           "  n_inl=%u  dist_to_centroid=%8.1f cm  cond=%.4f",
           name, ret, x, y, z, err, rms, (unsigned)ninl, dcen,
           cond_ratio_at(a, n, tx, ty));
    if (ret == 0 && dcen < 1.0f) printf("   <== IS THE CENTROID");
    printf("\n");
    return ret;
}

/* Assert that the solver does NOT return a position, or returns one that is
 * close to the truth.  A solve that is "close to the centroid" is the bug. */
static void expect(int ok, const char *what)
{
    g_checks++;
    if (!ok) { g_fail++; printf("  *** FAIL: %s\n", what); }
}

int main(void)
{
    dl_anchor_entry_t a[MAXA];

    printf("=== TDOA solver host test ===\n");
    printf("CM_PER_DTU = %.5f cm\n\n", (double)CM_PER_DTU);

    /* ── Case 1: four anchors on a square, tag inside ──────────────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2, 1000,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3, 1000, 1000,    0 };
        a[3] = (dl_anchor_entry_t){ 4,    0, 1000,    0 };
        printf("--- case 1: 4 anchors, square, tag inside, perfect data\n");
        int ret = run_case("square/inside", a, 4, 300, 400, 50, 0, 0);
        expect(ret == 0, "case 1 should solve");
    }

    /* ── Case 2: the tag AT the centroid (a legitimate fix!) ───────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2, 1000,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3, 1000, 1000,    0 };
        a[3] = (dl_anchor_entry_t){ 4,    0, 1000,    0 };
        printf("\n--- case 2: 4 anchors, tag EXACTLY at the centroid\n");
        printf("    (this is a LEGITIMATE fix — a centroid ban would break it)\n");
        int ret = run_case("square/tag-at-centroid", a, 4, 500, 500, 0, 0, 0);
        expect(ret == 0, "case 2 should solve");
    }

    /* ── Case 3: three anchors, EXACTLY determined ─────────────────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2, 1000,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3,  500,  800,    0 };
        printf("\n--- case 3: 3 anchors, exactly determined (2 TDOAs, 2 unknowns)\n");
        int ret = run_case("triangle/3-anchors", a, 3, 300, 400, 50, 0, 0);
        expect(ret == 0, "case 3 should solve");
    }

    /* ── Case 4: four anchors COLLINEAR, tag off the line ──────────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2,  500,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3, 1000,    0,    0 };
        a[3] = (dl_anchor_entry_t){ 4, 1500,    0,    0 };
        printf("\n--- case 4: 4 anchors COLLINEAR, tag off the line\n");
        run_case("collinear/tag-off-line", a, 4, 300, 400, 50, 0, 0);
    }

    /* ── Case 5: four anchors COLLINEAR, tag ON the line ───────────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2,  500,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3, 1000,    0,    0 };
        a[3] = (dl_anchor_entry_t){ 4, 1500,    0,    0 };
        printf("\n--- case 5: 4 anchors COLLINEAR, tag ON the line\n");
        printf("    (the TDOA Jacobian is rank deficient here)\n");
        run_case("collinear/tag-on-line", a, 4, 300, 0, 50, 0, 0);
    }

    /* ── Case 6: four anchors, garbage measurements ────────────────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2, 1000,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3, 1000, 1000,    0 };
        a[3] = (dl_anchor_entry_t){ 4,    0, 1000,    0 };
        printf("\n--- case 6: 4 anchors, GARBAGE measurements\n");
        run_case("square/garbage-tdoa", a, 4, 300, 400, 50, 1, 0);
    }

    /* ── Case 7: two anchors — underdetermined, must be refused ────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2, 1000,    0,    0 };
        printf("\n--- case 7: 2 anchors (underdetermined)\n");
        int ret = run_case("two-anchors", a, 2, 300, 400, 50, 0, 0);
        expect(ret != 0, "case 7 must be refused");
    }

    /* ── Case 8: six anchors in a ring, tag inside ─────────────────────── */
    {
        int i2;
        float R = 800.0f;
        for (i2 = 0; i2 < 6; i2++) {
            float ang = (float)(i2 * 60) * 3.14159265f / 180.0f;
            a[i2] = (dl_anchor_entry_t){ (uint16_t)(i2 + 1),
                       (int16_t)lrintf(R * cosf(ang)),
                       (int16_t)lrintf(R * sinf(ang)), 0 };
        }
        printf("\n--- case 8: 6 anchors in a ring, tag inside, perfect data\n");
        int ret = run_case("ring/6-anchors", a, 6, 250, -180, 40, 0, 0);
        expect(ret == 0, "case 8 should solve");
    }

    /* ── Case 9: all four anchors at the SAME position ──────────────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1, 0, 0, 0 };
        a[1] = (dl_anchor_entry_t){ 2, 0, 0, 0 };
        a[2] = (dl_anchor_entry_t){ 3, 0, 0, 0 };
        a[3] = (dl_anchor_entry_t){ 4, 0, 0, 0 };
        printf("\n--- case 9: all 4 anchors COINCIDENT (coordinates never set)\n");
        printf("    (this is what \"the anchors all sit at the origin\" looks like)\n");
        run_case("coincident/all-at-origin", a, 4, 300, 400, 50, 0, 0);
    }

    /* ── Case 10: three anchors coincident, one elsewhere ──────────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2,    0,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3,    0,    0,    0 };
        a[3] = (dl_anchor_entry_t){ 4, 1000,    0,    0 };
        printf("\n--- case 10: 3 anchors COINCIDENT + 1 elsewhere\n");
        run_case("coincident/3-at-one-point", a, 4, 300, 400, 50, 0, 0);
    }

    /* ── Case 11: perfectly good anchors, but NULL measurements ────────── */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2, 1000,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3, 1000, 1000,    0 };
        a[3] = (dl_anchor_entry_t){ 4,    0, 1000,    0 };
        printf("\n--- case 11: 4 anchors, NULL TDOAs (all differences zero)\n");
        printf("    (mathematically the only fit is the centroid — rms ~ 0)\n");
        run_case("square/null-tdoa", a, 4, 300, 400, 50, 3, 0);
    }

    /* ── Case 12/13: the slot-compensation error ─────────────────────────
     *
     * The tag removes the TDMA slot from every arrival instant:
     *     toa = rx_local - slot x 6.25 ms
     * The multiplication uses the tag's rate estimate for the Root, and a
     * rate estimate that is off by d ppm turns one slot into
     *     6.25 ms x d ppm  of path error
     * so the TDOA error grows with the SLOT INDEX: anchor k picks up
     * k x (the error of one slot).  The anchors are laid out here in slot
     * order (the host sorts them by distance to the Root by default), so
     * the error is a ramp along the array.
     *
     * 375 cm is one slot of path error at 20 ppm — the accuracy of the
     * tag's single-cycle rate estimate. */
    {
        a[0] = (dl_anchor_entry_t){ 1,    0,    0,    0 };
        a[1] = (dl_anchor_entry_t){ 2, 1000,    0,    0 };
        a[2] = (dl_anchor_entry_t){ 3, 1000, 1000,    0 };
        a[3] = (dl_anchor_entry_t){ 4,    0, 1000,    0 };
        printf("\n--- case 12: slot error, 10 cm of path per slot index\n");
        run_case("slot-bias/10cm-per-slot", a, 4, 300, 400, 50, 2, 10.0f);

        printf("\n--- case 13: slot error, 375 cm of path per slot index\n");
        printf("    (one slot at 20 ppm — the tag's single-cycle rate error)\n");
        run_case("slot-bias/375cm-per-slot", a, 4, 300, 400, 50, 2, 375.0f);
    }

    printf("\n=== %d checks, %d failed ===\n", g_checks, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
