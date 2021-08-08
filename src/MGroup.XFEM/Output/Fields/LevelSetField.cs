using System.Collections.Generic;
using System.Linq;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Output.Mesh;

namespace MGroup.XFEM.Output.Fields
{
    public class LevelSetField
    {
        private readonly XModel<IXMultiphaseElement> model;
        private readonly IClosedGeometry levelSet;
        private readonly ContinuousOutputMesh outMesh;

        public LevelSetField(IXModel model, IClosedGeometry levelSet)
        {
            this.levelSet = levelSet;
            this.outMesh = new ContinuousOutputMesh(model.Nodes.Values.OrderBy(n => n.ID).ToList(), model.EnumerateElements());
        }

        public LevelSetField(IXModel model, IClosedGeometry levelSet, ContinuousOutputMesh outputMesh)
        {
            this.levelSet = levelSet;
            this.outMesh = outputMesh;
        }

        public IOutputMesh Mesh => outMesh;

        public IEnumerable<double> CalcValuesAtVertices()
        {
            var vals = new double[outMesh.NumOutVertices];
            int idx = 0;
            foreach (XNode node in outMesh.OriginalVertices) // same order as mesh.OutVertices
            {
                vals[idx++] = levelSet.SignedDistanceOf(node);
            }
            return vals;
        }
    }
}
