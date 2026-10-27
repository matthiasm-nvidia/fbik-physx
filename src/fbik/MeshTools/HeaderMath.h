// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

#pragma once

#include "MeshTools/Vec.h"

namespace MeshTools
{

// -----------------------------------------------------------------------------------

CUDA_CALLABLE inline void headerJacobiRotate(Mat33& A, Mat33& R, int p, int q)
{
    // rotates A through phi in pq-plane to set A(p,q) = 0
    // rotation stored in R whose columns are eigenvectors of A
    if (A(p, q) == 0.0f)
        return;

    float d = (A(p, p) - A(q, q)) / (2.0f * A(p, q));
    float t = 1.0f / (fabsf(d) + sqrtf(d * d + 1.0f));
    if (d < 0.0f)
        t = -t;
    float c = 1.0f / sqrtf(t * t + 1);
    float s = t * c;
    A(p, p) += t * A(p, q);
    A(q, q) -= t * A(p, q);
    A(p, q) = A(q, p) = 0.0f;
    // transform A
    int k;
    for (k = 0; k < 3; k++)
    {
        if (k != p && k != q)
        {
            float Akp = c * A(k, p) + s * A(k, q);
            float Akq = -s * A(k, p) + c * A(k, q);
            A(k, p) = A(p, k) = Akp;
            A(k, q) = A(q, k) = Akq;
        }
    }
    // store rotation in R
    for (k = 0; k < 3; k++)
    {
        float Rkp = c * R(k, p) + s * R(k, q);
        float Rkq = -s * R(k, p) + c * R(k, q);
        R(k, p) = Rkp;
        R(k, q) = Rkq;
    }
}

// ----------------------------------------------------------------------------------------------

CUDA_CALLABLE inline void headerEigenDecomposition(const Mat33& A, Mat33& eigenVecs, Vec3& eigenVals, bool sorted = true)
{
    const int numJacobiIterations = 10;
    const float epsilon = 1e-6f;

    Mat33 D = A;

    // only for symmetric matrices!
    eigenVecs = Mat33(Identity); // unit matrix
    int iter = 0;
    while (iter < numJacobiIterations)
    { // 3 off diagonal elements
      // find off diagonal element with maximum modulus
        int p, q;
        float a, max;
        max = (float)fabs(D(0, 1));
        p = 0;
        q = 1;
        a = (float)fabs(D(0, 2));
        if (a > max)
        {
            p = 0;
            q = 2;
            max = a;
        }
        a = (float)fabs(D(1, 2));
        if (a > max)
        {
            p = 1;
            q = 2;
            max = a;
        }
        // all small enough -> done
        if (max < epsilon)
            break;
        // rotate matrix with respect to that element
        headerJacobiRotate(D, eigenVecs, p, q);
        iter++;
    }
    eigenVals[0] = D(0, 0);
    eigenVals[1] = D(1, 1);
    eigenVals[2] = D(2, 2);

    if (sorted)
    {
        if (fabs(eigenVals[0]) < fabs(eigenVals[1]))
        {
            float e = eigenVals[0];
            eigenVals[0] = eigenVals[1];
            eigenVals[1] = e;
            Vec3 v = eigenVecs.column0;
            eigenVecs.column0 = eigenVecs.column1;
            eigenVecs.column1 = v;
        }
        if (fabs(eigenVals[1]) < fabs(eigenVals[2]))
        {
            float e = eigenVals[1];
            eigenVals[1] = eigenVals[2];
            eigenVals[2] = e;
            Vec3 v = eigenVecs.column1;
            eigenVecs.column1 = eigenVecs.column2;
            eigenVecs.column2 = v;
        }
        if (fabs(eigenVals[0]) < fabs(eigenVals[1]))
        {
            float e = eigenVals[0];
            eigenVals[0] = eigenVals[1];
            eigenVals[1] = e;
            Vec3 v = eigenVecs.column0;
            eigenVecs.column0 = eigenVecs.column1;
            eigenVecs.column1 = v;
        }
        eigenVecs.column2 = eigenVecs.column0.cross(eigenVecs.column1);
    }
}

//---------------------------------------------------------------------


} // namespace MeshTools
