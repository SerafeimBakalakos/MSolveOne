using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;

namespace MGroup.XFEM.Output.Fields
{
    public class StrainsStressesAtGaussPointsField
    {
        private readonly XModel<IXMultiphaseElement> model;

        public StrainsStressesAtGaussPointsField(XModel<IXMultiphaseElement> model)
        {
            this.model = model;
        }

        public (Dictionary<double[], double[]> strains, Dictionary<double[], double[]> stresses) 
            CalcTensorsAtPoints(IVectorView solution)
        {
            if (model.Subdomains.Count != 1) throw new NotImplementedException();
            XSubdomain subdomain = model.Subdomains.First().Value;

            var allStrains = new Dictionary<double[], double[]>();
            var allStresses = new Dictionary<double[], double[]>();
            foreach (IXStructuralMultiphaseElement element in model.Elements)
            {
                IList<double[]> elementDisplacements = Utilities.ExtractElementDisplacements(element, subdomain, solution);
                HashSet<IEnrichmentFunction> elementEnrichments = element.FindEnrichments();
                foreach (GaussPoint gp in element.BulkIntegrationPoints)
                {
                    XPoint point = StrainStressField.PreparePoint(gp.Coordinates, element);
                    double[] coordsCartesian =
                        Utilities.TransformNaturalToCartesian(point.ShapeFunctions, element.Nodes);
                    (double[] strains, double[] stresses) = StrainStressField.CalcStrainsStressesAt(
                        point, element, elementDisplacements, elementEnrichments);
                    allStrains[coordsCartesian] = strains;
                    allStresses[coordsCartesian] = stresses;
                }
            }
            return (allStrains, allStresses);
        }
    }
}
