#include "mcue/kin.h"

#include <math.h>
#include <stddef.h>

#include "mcue/linalg.h"

void mc_rotation(const mc_pose *q, double R[3][3])
{
	double cx = cos(q->roll), sx = sin(q->roll);
	double cy = cos(q->pitch), sy = sin(q->pitch);
	double cz = cos(q->yaw), sz = sin(q->yaw);

	/* Rz(yaw) * Ry(pitch) * Rx(roll) */
	R[0][0] = cz * cy;
	R[0][1] = (cz * sy * sx) - (sz * cx);
	R[0][2] = (cz * sy * cx) + (sz * sx);
	R[1][0] = sz * cy;
	R[1][1] = (sz * sy * sx) + (cz * cx);
	R[1][2] = (sz * sy * cx) - (cz * sx);
	R[2][0] = -sy;
	R[2][1] = cy * sx;
	R[2][2] = cy * cx;
}

static mc_vec3 mat_vec(const double R[3][3], const mc_vec3 *v)
{
	return (mc_vec3){
		(R[0][0] * v->x) + (R[0][1] * v->y) + (R[0][2] * v->z),
		(R[1][0] * v->x) + (R[1][1] * v->y) + (R[1][2] * v->z),
		(R[2][0] * v->x) + (R[2][1] * v->y) + (R[2][2] * v->z),
	};
}

static mc_vec3 cross(const mc_vec3 *a, const mc_vec3 *b)
{
	return (mc_vec3){
		(a->y * b->z) - (a->z * b->y),
		(a->z * b->x) - (a->x * b->z),
		(a->x * b->y) - (a->y * b->x),
	};
}

/* Leg vector L_i = p + p0 + R b_i - a_i, and r_i = R b_i. */
static void leg_vector(const mc_geometry *g, const mc_pose *q, const double R[3][3], int i,
		       mc_vec3 *L, mc_vec3 *r)
{
	*r = mat_vec(R, &g->plat[i]);
	L->x = q->x + r->x - g->base[i].x;
	L->y = q->y + r->y - g->base[i].y;
	L->z = q->z + g->h0 + r->z - g->base[i].z;
}

static double norm(const mc_vec3 *v)
{
	return sqrt((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
}

void mc_ik(const mc_geometry *g, const mc_pose *q, double legs[MC_LEGS])
{
	double R[3][3];

	mc_rotation(q, R);
	for (int i = 0; i < MC_LEGS; i++) {
		mc_vec3 L, r;

		leg_vector(g, q, R, i, &L, &r);
		legs[i] = norm(&L);
	}
}

void mc_jacobian(const mc_geometry *g, const mc_pose *q, double J[MC_LEGS][6])
{
	double R[3][3];

	mc_rotation(q, R);

	/* omega = E * (roll_dot, pitch_dot, yaw_dot) for R = Rz Ry Rx (base frame). */
	double cp = cos(q->pitch), sp = sin(q->pitch);
	double cy = cos(q->yaw), sy = sin(q->yaw);
	const mc_vec3 e_roll = {cy * cp, sy * cp, -sp};
	const mc_vec3 e_pitch = {-sy, cy, 0.0};
	const mc_vec3 e_yaw = {0.0, 0.0, 1.0};

	for (int i = 0; i < MC_LEGS; i++) {
		mc_vec3 L, r;

		leg_vector(g, q, R, i, &L, &r);
		double l = norm(&L);
		mc_vec3 n = {L.x / l, L.y / l, L.z / l};
		mc_vec3 rxn = cross(&r, &n);

		J[i][0] = n.x;
		J[i][1] = n.y;
		J[i][2] = n.z;
		J[i][3] = (rxn.x * e_roll.x) + (rxn.y * e_roll.y) + (rxn.z * e_roll.z);
		J[i][4] = (rxn.x * e_pitch.x) + (rxn.y * e_pitch.y) + (rxn.z * e_pitch.z);
		J[i][5] = (rxn.x * e_yaw.x) + (rxn.y * e_yaw.y) + (rxn.z * e_yaw.z);
	}
}

bool mc_fk(const mc_geometry *g, const double legs[MC_LEGS], mc_pose *q, int max_iter, double tol,
	   mc_fk_info *info)
{
	double J[MC_LEGS][6];
	double l[MC_LEGS];
	double err[MC_LEGS];
	double dq[6];
	int it = 0;
	bool done = false;

	while ((it < max_iter) && !done) {
		mc_ik(g, q, l);
		for (int i = 0; i < MC_LEGS; i++) {
			err[i] = legs[i] - l[i];
		}
		mc_jacobian(g, q, J);
		if (!mc_solve6(J, err, dq)) {
			break; /* singular: keep the last good iterate */
		}
		q->x += dq[0];
		q->y += dq[1];
		q->z += dq[2];
		q->roll += dq[3];
		q->pitch += dq[4];
		q->yaw += dq[5];
		it++;

		double step = 0.0;

		for (int k = 0; k < 6; k++) {
			step = fmax(step, fabs(dq[k]));
		}
		done = (step < tol);
	}

	mc_ik(g, q, l);
	double res = 0.0;

	for (int i = 0; i < MC_LEGS; i++) {
		res = fmax(res, fabs(legs[i] - l[i]));
	}
	if (info != NULL) {
		info->iterations = it;
		info->residual = res;
		info->converged = done || (res < tol);
	}
	return done || (res < tol);
}

bool mc_statics(const mc_geometry *g, const mc_pose *q, double mass, const mc_vec3 *com,
		double force[MC_LEGS])
{
	double R[3][3];
	double JtT[6][6]; /* transpose of the twist Jacobian: columns are [n_i; r_i x n_i] */

	mc_rotation(q, R);
	for (int i = 0; i < MC_LEGS; i++) {
		mc_vec3 L, r;

		leg_vector(g, q, R, i, &L, &r);
		double l = norm(&L);
		mc_vec3 n = {L.x / l, L.y / l, L.z / l};
		mc_vec3 rxn = cross(&r, &n);

		JtT[0][i] = n.x;
		JtT[1][i] = n.y;
		JtT[2][i] = n.z;
		JtT[3][i] = rxn.x;
		JtT[4][i] = rxn.y;
		JtT[5][i] = rxn.z;
	}

	/* The legs must supply +m g along z and the matching moment about the platform origin. */
	mc_vec3 w = {0.0, 0.0, mass * MC_GRAVITY};
	mc_vec3 c = mat_vec(R, com);
	mc_vec3 m = cross(&c, &w);
	double W[6] = {w.x, w.y, w.z, m.x, m.y, m.z};

	return mc_solve6(JtT, W, force);
}
