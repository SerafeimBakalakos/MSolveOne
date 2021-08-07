using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Exceptions;

namespace MGroup.XFEM.Geometry.LSM
{
    public class UnionLsm3D : SimpleLsm3D
    {
        private readonly List<SimpleLsm3D> mergedLsms;

        public UnionLsm3D(int id, IReadOnlyList<XNode> nodes, SimpleLsm3D starterLsm) : base(id, starterLsm.NodalLevelSets)
        {
            this.mergedLsms = new List<SimpleLsm3D>();
            mergedLsms.Add(starterLsm);
        }

        public override IElementDiscontinuityInteraction Intersect(IXFiniteElement element)
        {
            try
            {
                return base.Intersect(element);
            }
            catch (InvalidElementGeometryIntersectionException)
            {
                var unionIntersectionMesh = new IntersectionMesh3D_OLD();
                foreach (SimpleLsm3D lsm in mergedLsms)
                {
                    Dictionary<int, double> levelSetSubset = FindLevelSetsOfElementNodes(element, lsm.NodalLevelSets);
                    RelativePositionCurveElement position = FindRelativePosition(element, levelSetSubset);
                    if (position == RelativePositionCurveElement.Disjoint) continue;
                    else if (position == RelativePositionCurveElement.Conforming)
                    {
                        throw new InvalidElementGeometryIntersectionException(
                            "If one merged LSM is conforming, then no other one must intersect the element");
                    }
                    else if (position == RelativePositionCurveElement.Intersecting)
                    {
                        var intersectionMesh = FindInteractionIntersecting(element, levelSetSubset);
                        unionIntersectionMesh.MergeWith(intersectionMesh);
                    }
                    else throw new NotImplementedException();
                }
                if (unionIntersectionMesh.Vertices.Count > 0)
                {
                    return new LsmElementIntersection3D(ID, 
                        RelativePositionCurveElement.Intersecting, element, unionIntersectionMesh);
                }
                else throw new InvalidElementGeometryIntersectionException("None merged LSM intersects the element");
            }
            
        }

        public override void UnionWith(IClosedGeometry otherGeometry)
        {
            // If there is only one level set array, clone it, as to not corrupt the already merged curve
            if (mergedLsms.Count == 1)
            {
                var clone = new double[this.NodalLevelSets.Length];
                Array.Copy(this.NodalLevelSets, clone, this.NodalLevelSets.Length);
                this.NodalLevelSets = clone;
            }

            if (otherGeometry is UnionLsm3D otherUnionLsm)
            {
                MergeNodalLevelSets(otherUnionLsm.NodalLevelSets);
                this.mergedLsms.AddRange(otherUnionLsm.mergedLsms);
            }
            else if (otherGeometry is SimpleLsm3D otherLsm)
            {
                MergeNodalLevelSets(otherLsm.NodalLevelSets);
                mergedLsms.Add(otherLsm);
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
        }

        private void MergeNodalLevelSets(double[] otherNodalLevelSets)
        {
            if (this.NodalLevelSets.Length != otherNodalLevelSets.Length)
            {
                throw new ArgumentException("Incompatible Level Set geometry");
            }
            for (int i = 0; i < this.NodalLevelSets.Length; ++i)
            {
                this.NodalLevelSets[i] = Math.Min(this.NodalLevelSets[i], otherNodalLevelSets[i]);
            }
        }
    }
}
