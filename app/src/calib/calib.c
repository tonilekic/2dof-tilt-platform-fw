/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include "calib.h"

/* Accumulation and the 4×4 solve run in double precision. This is a one-shot
 * calibration, so the software-double cost on the M4F is irrelevant, and it
 * keeps the normal-equation moments (which sum squares of ~g-sized terms)
 * from losing conditioning. Inputs and the stored map stay float. */

void calib_fit_reset(struct calib_fit *f)
{
	memset(f, 0, sizeof(*f));
}

void calib_fit_add(struct calib_fit *f, const float raw[3],
		   const float target[3])
{
	const double x[4] = { raw[0], raw[1], raw[2], 1.0 };

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			f->N[i][j] += x[i] * x[j];
		}
	}

	for (int k = 0; k < 3; k++) {
		const double g = target[k];

		for (int i = 0; i < 4; i++) {
			f->b[k][i] += x[i] * g;
		}
		f->gg[k] += g * g;
	}

	f->n++;
}

int calib_fit_solve(const struct calib_fit *f, struct calib *out,
		    float *resid_rms)
{
	if (f->n < CALIB_MIN_POSES) {
		return -EINVAL;
	}

	/* Augmented system [ N | b₀ b₁ b₂ ]: one 4×4 elimination resolves all
	 * three output rows at once because they share N. */
	double m[4][7];

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m[i][j] = f->N[i][j];
		}
		m[i][4] = f->b[0][i];
		m[i][5] = f->b[1][i];
		m[i][6] = f->b[2][i];
	}

	/* Gauss-Jordan with partial pivoting. */
	for (int col = 0; col < 4; col++) {
		int piv = col;
		double best = fabs(m[col][col]);

		for (int r = col + 1; r < 4; r++) {
			if (fabs(m[r][col]) > best) {
				best = fabs(m[r][col]);
				piv = r;
			}
		}

		/* N is Σ xxᵀ with entries O(n·g²); a pivot this small means the
		 * pose set doesn't span the 4 parameters (e.g. all poses share a
		 * raw-axis value, or fewer than 4 distinct directions). */
		if (best < 1.0e-9) {
			return -EIO;
		}

		if (piv != col) {
			for (int c = 0; c < 7; c++) {
				double tmp = m[col][c];

				m[col][c] = m[piv][c];
				m[piv][c] = tmp;
			}
		}

		double d = m[col][col];

		for (int c = 0; c < 7; c++) {
			m[col][c] /= d;
		}

		for (int r = 0; r < 4; r++) {
			if (r == col) {
				continue;
			}
			double factor = m[r][col];

			for (int c = 0; c < 7; c++) {
				m[r][c] -= factor * m[col][c];
			}
		}
	}

	/* Solution columns: axis k's parameter i is m[i][4 + k]. */
	for (int k = 0; k < 3; k++) {
		for (int i = 0; i < 4; i++) {
			out->A[k][i] = (float)m[i][4 + k];
		}
	}
	out->valid = true;

	if (resid_rms != NULL) {
		/* Per-axis residual sum of squares from accumulated moments only:
		 *   RSSₖ = Σ gₖ²  −  Aₖ · (Σ xₚ gₖ,ₚ) = gg[k] − Aₖ·b[k].
		 * Summed over the 3 axes and divided by n, this is the mean
		 * squared residual *vector* magnitude; its root is the RMS accel
		 * error of the fit in m/s². */
		double rss = 0.0;

		for (int k = 0; k < 3; k++) {
			double dot = 0.0;

			for (int i = 0; i < 4; i++) {
				dot += (double)out->A[k][i] * f->b[k][i];
			}
			double r = f->gg[k] - dot;

			rss += (r > 0.0) ? r : 0.0; /* clamp tiny negatives */
		}

		*resid_rms = (float)sqrt(rss / (double)f->n);
	}

	return 0;
}

void calib_identity(struct calib *out)
{
	memset(out->A, 0, sizeof(out->A));
	out->A[0][0] = 1.0f;
	out->A[1][1] = 1.0f;
	out->A[2][2] = 1.0f;
	out->valid = true;
}

void calib_apply(const struct calib *c, const float a_in[3], float a_out[3])
{
	if (!c->valid) {
		a_out[0] = a_in[0];
		a_out[1] = a_in[1];
		a_out[2] = a_in[2];
		return;
	}

	/* Latch inputs so the map is safe to apply in place. */
	const float x = a_in[0], y = a_in[1], z = a_in[2];

	for (int k = 0; k < 3; k++) {
		a_out[k] = c->A[k][0] * x + c->A[k][1] * y + c->A[k][2] * z +
			   c->A[k][3];
	}
}
