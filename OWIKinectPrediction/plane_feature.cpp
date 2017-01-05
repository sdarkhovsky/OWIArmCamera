#include <Eigen/Eigenvalues>
#include "point_cloud.h"
#include "plane_feature.h"

using namespace ais;
using namespace Eigen;

const float eps_plane_normal_extent = 0.00001f;

// todo: consider using RANSAC to detect a plane locally
bool detect_local_plane_feature( c_point_cloud& point_cloud, size_t u, size_t v, c_plane_feature& plane_feature) {

	const unsigned int num_pnts = 9;
	size_t i, r, c;

    if (u <= 0 || u >= point_cloud.points.size() - 1 || v <= 0 || v >= point_cloud.points[u].size() - 1)
        return false;

	// calculate PCA on the neighbourhood points
	// see http://docs.opencv.org/3.1.0/d1/dee/tutorial_introduction_to_pca.html

    MatrixXf X(num_pnts,3);

    i = 0;
	// Organize the data set
	for (r = u - 1; r <= u + 1; ++r) {
		for (c = v - 1; c <= v + 1; ++c) {
			assert(i < num_pnts);
			X(i,0) = point_cloud.points[r][c].X;
			X(i,1) = point_cloud.points[r][c].Y;
			X(i,2) = point_cloud.points[r][c].Z;
			i++;
		}
	}

	// Calculate the empirical mean
	Vector3f mean;
	for (r = 0; r < num_pnts; r++) {
		for (c = 0; c < 3; c++) {
			mean(c) += X(r, c);
		}
	}

	// Calculate the deviations from the mean
	VectorXf h(num_pnts);
	for (i = 0; i < num_pnts; i++) {
		h(i) = 1;
	}

	MatrixXf B(num_pnts, 3);
	B = X - h * mean.transpose();

	// Find the covariance matrix
	Matrix3f C;
	C = B.transpose() * B;
    C /= (num_pnts - 1);

	// Find the eigenvectors and eigenvalues of the covariance matrix
    EigenSolver<MatrixXf> es(C);
//    es.compute(C);

    // return value is the matrix of complex
    auto eigen_values = es.eigenvalues();
    auto eigen_vectors = es.eigenvectors();

    // find minimal eigen value and its eigen vector
    r = 0;
    float min_eigen_val = std::abs(eigen_values(0));
    for (i = 1; i < 3; i++) {
        float eigen_val = std::abs(eigen_values(i));
        if (eigen_val < min_eigen_val) {
            min_eigen_val = eigen_val;
            r = i;
        }
    }
    auto plane_normal = eigen_vectors.col(r);

    if (min_eigen_val < eps_plane_normal_extent) {
        for (i = 1; i < 3; i++) {
            plane_feature.normal.elem[i] = plane_normal(i).real();
        }
        int r[2], c[2], j;
        c_line_segment_feature line_segment;
        for (i = 0; i < 3; i++) {
            switch (i) {
                case 0:
                    r[0] = u - 1; c[0] = v - 1; r[1] = u - 1; c[1] = v + 1;
                    break;
            }
            for (j = 0; j < 2; j++) {
                line_segment.ends[j] =
                    c_vector3f(point_cloud.points[r[j]][c[j]].X, point_cloud.points[r[j]][c[j]].Y, point_cloud.points[r[j]][c[j]].Z);
            }

            plane_feature.boundary.elements.push_back(line_segment);
        }

        return true;
    }

    return false;
}

bool detect_plane_features(c_point_cloud& point_cloud, vector<c_plane_feature>& plane_features) {
    size_t u, v;
    size_t num_point_cloud_rows = point_cloud.points.size();
    if (num_point_cloud_rows <= 0)
        return false;
    size_t num_point_cloud_cols = point_cloud.points[0].size();

    plane_features.clear();

    for (u = 1; u < num_point_cloud_rows - 1; u = u + 2) {
        for (v = 1; v < num_point_cloud_cols - 1; v = v + 2) {
            c_plane_feature plane_feature;
            if (detect_local_plane_feature(point_cloud, u, v, plane_feature)) {

            }
        }
    }
    return true;
}
