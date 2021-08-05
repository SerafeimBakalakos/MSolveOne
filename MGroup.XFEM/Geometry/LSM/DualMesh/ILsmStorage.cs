using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    public interface ILsmStorage
    {
        int Dimension { get; }

        double GetLevelSet(int nodeID);

        void Initialize(IClosedManifold originalGeometry, IStructuredMesh mesh); //TODO: Also for unstruvtured meshes

        bool OverlapsWith(ILsmStorage otherGeometry);

        void UnionWith(ILsmStorage otherGeometry);
    }
}
