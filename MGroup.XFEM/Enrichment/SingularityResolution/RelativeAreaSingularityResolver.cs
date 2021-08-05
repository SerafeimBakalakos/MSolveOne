using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;

//TODO: The tolerance is counter-intuitive. Larger tolerance should mean more nodes are enriched with Heaviside.
namespace MGroup.XFEM.Enrichment.SingularityResolution
{
    /// <summary>
    /// Must not be used unless the elements are decomposed into subcells that conform to the discontinuities.
    /// </summary>
    public class RelativeAreaSingularityResolver : ISingularityResolver
    {
        private readonly double relativeAreaTolerance;

        public RelativeAreaSingularityResolver(double relativeAreaTolerance = 1E-4)
        {
            this.relativeAreaTolerance = relativeAreaTolerance;
        }

        /// <summary>
        /// Given a set of step enriched nodes, find which of them must not be enriched, in order to avoid the global
        /// stiffness matrix being singular.
        /// </summary>
        /// <param name="mesh"></param>
        /// <param name="stepNodes">They will not be altered.</param>
        public HashSet<XNode> FindStepEnrichedNodesToRemove(IEnumerable<XNode> stepNodes, IXGeometryDescription boundary)
        {
            bool intersectedElementsExist = false;
            var processedElements = new Dictionary<IXFiniteElement, (double, double)>();
            var nodesToRemove = new HashSet<XNode>();
            foreach (XNode node in stepNodes)
            {
                double nodeBulkSizePos = 0.0;
                double nodeBulkSizeNeg = 0.0;

                foreach (IXFiniteElement element in node.ElementsDictionary.Values)
                {
                    bool alreadyProcessed = processedElements.TryGetValue(element, out (double pos, double neg) elementBulkSize);
                    if (!alreadyProcessed)
                    {
                        elementBulkSize = FindSignedAreasOfElement(element, boundary, ref intersectedElementsExist);
                        processedElements[element] = elementBulkSize;
                    }
                    nodeBulkSizePos += elementBulkSize.pos;
                    nodeBulkSizeNeg += elementBulkSize.neg;
                }

                double nodalDistance = boundary.SignedDistanceOf(node);
                if (nodalDistance >= 0)
                {
                    double negRatio = nodeBulkSizeNeg / (nodeBulkSizePos + nodeBulkSizeNeg);
                    if (negRatio < relativeAreaTolerance) nodesToRemove.Add(node);
                }
                else
                {
                    double posRatio = nodeBulkSizePos / (nodeBulkSizePos + nodeBulkSizeNeg);
                    if (posRatio < relativeAreaTolerance) nodesToRemove.Add(node);
                }
            }

            if (!intersectedElementsExist)
            {
                Debug.WriteLine(typeof(RelativeAreaSingularityResolver).Name 
                    + ": Could not find any elements intersected by discontinuities");
            }
            return nodesToRemove;
        }

        // TODO: I should really cache these somehow, so that they can be accessible from the crack object. They are used at various points.
        private (double totalArea1, double totalArea2) FindSignedAreasOfElement(IXFiniteElement element,
            IXGeometryDescription boundary, ref bool intersectedElementsExist)
        {
            double totalBulkSizePos = 0.0;
            double totalBulkSizeNeg = 0.0;

            if ((element.ConformingSubcells != null) && (element.ConformingSubcells.Length != 0))
            {
                intersectedElementsExist = true;
                IElementSubcell[] subcells = element.ConformingSubcells;
                foreach (IElementSubcell subcell in subcells)
                {
                    // Calculate its area/volume and on which side it lies, based on its centroid
                    double[] centroidNatural = subcell.FindCentroidNatural();
                    (double[] centroidCartesian, double bulkSize) = subcell.FindCentroidAndBulkSizeCartesian(element);
                    XPoint centroid = new XPoint(centroidNatural.Length);
                    centroid.Element = element;
                    centroid.Coordinates[CoordinateSystem.ElementNatural] = centroidNatural;
                    centroid.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(centroidNatural);

                    double signedDistance = boundary.SignedDistanceOf(centroid);
                    if (signedDistance >= 0) totalBulkSizePos += bulkSize;
                    else totalBulkSizeNeg += bulkSize;
                }
            }
            else
            {
                // Calculate the area/volume of the whole element and on which side it lies, based on its centroid
                double bulkSize = element.CalcBulkSizeCartesian();
                double[] centroidNatural = element.FindCentroidNatural();
                XPoint centroid = new XPoint(centroidNatural.Length);
                centroid.Element = element;
                centroid.Coordinates[CoordinateSystem.ElementNatural] = centroidNatural;
                centroid.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(centroidNatural);

                double signedDistance = boundary.SignedDistanceOf(centroid);
                if (signedDistance >= 0) totalBulkSizePos += bulkSize;
                else totalBulkSizeNeg += bulkSize;
            }

            return (totalBulkSizePos, totalBulkSizeNeg);
        }
    }
}
