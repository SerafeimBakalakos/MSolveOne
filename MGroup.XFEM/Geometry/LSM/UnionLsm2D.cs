//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using ISAAR.MSolve.Geometry.Coordinates;
//using MGroup.XFEM.Elements;
//using MGroup.XFEM.Entities;

//namespace MGroup.XFEM.Geometry.LSM
//{
//    public class UnionLsm2D : SimpleLsm2D
//    {
//        private readonly List<SimpleLsm2D> mergedLsms;

//        public UnionLsm2D(int id, XModel physicalModel, SimpleLsm2D starterLsm) : base(id, starterLsm.NodalLevelSets)
//        {
//            this.mergedLsms = new List<SimpleLsm2D>();
//            mergedLsms.Add(starterLsm);
//        }

//        public override IElementGeometryIntersection Intersect(IXFiniteElement element)
//        {
//            // If thre are more than 2 intersection points, take the intersection of each merged level sets
//            throw new NotImplementedException();
//        }

//        //public double SignedDistanceOf(XNode node)
//        //{
//        //    double min = mergedLsms[0].SignedDistanceOf(node);
//        //    for (int i = 1; i < mergedLsms.Count; ++i)
//        //    {
//        //        min = Math.Min(min, mergedLsms[i].SignedDistanceOf(node));
//        //    }
//        //    return min;
//        //}

//        //public double SignedDistanceOf(XPoint point)
//        //{
//        //    IReadOnlyList<XNode> nodes = point.Element.Nodes;
//        //    var nodalLevelSets = new double[nodes.Count];
//        //    for (int n = 0; n < nodes.Count; ++n)
//        //    {
//        //        nodalLevelSets[n] = SignedDistanceOf(nodes[n]);
//        //    }

//        //    double[] shapeFunctions = point.ShapeFunctions;
//        //    double result = 0;
//        //    for (int n = 0; n < nodes.Count; ++n)
//        //    {
//        //        result += shapeFunctions[n] * nodalLevelSets[n];
//        //    }
//        //    return result;
//        //}

//        public override void UnionWith(IImplicitGeometry otherGeometry)
//        {
//            // If there is only one level set array, clone it, as to not corrupt the already merged curve
//            if (mergedLsms.Count == 1)
//            {
//                var clone = new double[this.NodalLevelSets.Length];
//                Array.Copy(this.NodalLevelSets, clone, this.NodalLevelSets.Length);
//            }

//            if (otherGeometry is UnionLsm2D otherUnionLsm)
//            {
//                MergeNodalLevelSets(otherUnionLsm.NodalLevelSets);
//                this.mergedLsms.AddRange(otherUnionLsm.mergedLsms);
//            }
//            else if (otherGeometry is SimpleLsm2D otherLsm)
//            {
//                MergeNodalLevelSets(otherLsm.NodalLevelSets);
//                mergedLsms.Add(otherLsm);
//            }
//            else throw new ArgumentException("Incompatible Level Set geometry");
//        }

//        private void MergeNodalLevelSets(double[] otherNodalLevelSets)
//        {
//            if (this.NodalLevelSets.Length != otherNodalLevelSets.Length)
//            {
//                throw new ArgumentException("Incompatible Level Set geometry");
//            }
//            for (int i = 0; i < this.NodalLevelSets.Length; ++i)
//            {
//                this.NodalLevelSets[i] = Math.Min(this.NodalLevelSets[i], otherNodalLevelSets[i]);
//            }
//        }
//    }
//}
