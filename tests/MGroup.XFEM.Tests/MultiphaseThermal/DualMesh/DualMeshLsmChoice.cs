using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.LSM.DualMesh;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Tests.MultiphaseThermal.DualMesh
{
    public enum DualMeshLsmChoice
    {
        Global, Local, Fixed
    }

    internal static class DualMeshLsmChoiceExtensions
    {
        internal static ILsmStorage Create(this DualMeshLsmChoice choice, int dimension)
        {
            if (choice == DualMeshLsmChoice.Global)
            {
                return new LsmStorageGlobal(dimension);
            }
            else if (choice == DualMeshLsmChoice.Local)
            {
                return new LsmStorageLocal(dimension);
            }
            else if (choice == DualMeshLsmChoice.Fixed)
            {
                return new LsmStorageFixed(dimension);
            }
            else
            {
                throw new NotImplementedException();
            }
        }

        internal static DualMeshLsm2DBase_OLD Create_OLD(this DualMeshLsmChoice choice, 
            int id, IDualMesh dualMesh, ICurve2D closedCurve)
        {
            if (choice == DualMeshLsmChoice.Global)
            {
                return new GlobalDualMeshLsm2D_OLD(id, dualMesh, closedCurve);
            }
            else if (choice == DualMeshLsmChoice.Local)
            {
                return new LocalDualMeshLsm2D_OLD(id, dualMesh, closedCurve);
            }
            else if (choice == DualMeshLsmChoice.Fixed)
            {
                return new FixedDualMeshLsm2D_OLD(id, dualMesh, closedCurve);
            }
            else
            {
                throw new NotImplementedException();
            }
        }

        internal static DualMeshLsm3DBase_OLD Create(this DualMeshLsmChoice choice,
            int id, DualCartesianMesh3D dualMesh, ISurface3D closedSurface)
        {
            if (choice == DualMeshLsmChoice.Global)
            {
                return new GlobalDualMeshLsm3D_OLD(id, dualMesh, closedSurface);
            }
            else if (choice == DualMeshLsmChoice.Local)
            {
                return new LocalDualMeshLsm3D_OLD(id, dualMesh, closedSurface);
            }
            else if (choice == DualMeshLsmChoice.Fixed)
            {
                return new FixedDualMeshLsm3D_OLD(id, dualMesh, closedSurface);
            }
            else
            {
                throw new NotImplementedException();
            }
        }
    }
}
