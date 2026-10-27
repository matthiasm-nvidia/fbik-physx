// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

// Adapted point-cloud PCA OBB path; unrelated geometry and convex optimization omitted.
#include "Geometry.h"
#include "HeaderMath.h"
namespace MeshTools
{
OBB computeOBBFromPoints(const std::vector<Vec3>& vertices)
{
    OBB obb = OBB(Empty);
    Vec3 center(Zero);
    Mat33 A(Zero);

    {
        size_t numVerts = (int)vertices.size();

        if (numVerts == 0)
        {
            return obb;
        }

        for (size_t i = 0; i < numVerts; i++)
        {
            center += vertices[i];
        }

        center /= float(numVerts);

        for (size_t i = 0; i < numVerts; i++)
        {
            const Vec3 r = vertices[i] - center;
            A += outerProduct(r, r);
        }
    }

    Mat33 frame;
    Vec3 eigenVals;

    headerEigenDecomposition(A, frame, eigenVals, true);

    frame.column0.normalize();
    frame.column1.normalize();
    frame.column2.normalize();

    Mat33 inv = frame.getInverse();
    Bounds3 localBounds;

    localBounds.setEmpty();

    for (size_t i = 0; i < vertices.size(); i++)
    {
        Vec3 q = inv * (vertices[i] - center);
        localBounds.include(q);
    }

    Vec3 comOffset = -localBounds.getCenter();

    // break the symmetry

    float eps = 0.0001f * localBounds.getDimensions().length();

    int s = 1;

    if (comOffset.x > eps)
    {
        frame.column0 = -frame.column0;
        s = -s;
    }
    else if (comOffset.x > -eps)
    {
        s = 0; // symmetric
    }
    if (comOffset.y > eps)
    {
        frame.column1 = -frame.column1;
        s = -s;
    }
    else if (comOffset.y > -eps)
    {
        s = 0;
    }
    if (comOffset.z > eps)
    {
        frame.column2 = -frame.column2;
        s = -s;
    }
    else if (comOffset.z > -eps)
    {
        s = 0;
    }

    if (frame.getDeterminant() < 0.0f) // force right handed frame
    {
        // swap the direction with the smallest offset (most symmetric)

        float c0 = fabsf(comOffset.x);
        float c1 = fabsf(comOffset.y);
        float c2 = fabsf(comOffset.z);

        if (c0 < c1 && c0 < c2)
        {
            frame.column0 = -frame.column0;
        }
        else if (c1 < c2)
        {
            frame.column1 = -frame.column1;
        }
        else
        {
            frame.column2 = -frame.column2;
        }
    }

    inv = frame.getInverse();
    localBounds.setEmpty();

    for (size_t i = 0; i < vertices.size(); i++)
    {
        Vec3 q = inv * vertices[i];
        localBounds.include(q);
    }
    obb.halfExtents = localBounds.getHalfExtents();
    obb.trans.q = Quat(frame);
    obb.trans.p = obb.trans.q.rotate(localBounds.getCenter());

    return obb;
}

}
