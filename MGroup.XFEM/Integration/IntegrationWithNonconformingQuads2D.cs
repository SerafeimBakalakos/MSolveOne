using System.Collections.Generic;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;

namespace MGroup.XFEM.Integration
{
    public class IntegrationWithNonconformingQuads2D : IBulkIntegration
    {
        private readonly int quadsPerAxis;
        private readonly GaussLegendre2D  gaussQuadrature;

        public IntegrationWithNonconformingQuads2D(int quadsPerAxis) : 
            this(quadsPerAxis, GaussLegendre2D.GetQuadratureWithOrder(2,2))
        {
        }

        public IntegrationWithNonconformingQuads2D(int quadsPerAxis, GaussLegendre2D quadratureInQuads)
        {
            this.quadsPerAxis = quadsPerAxis;
            this.gaussQuadrature = quadratureInQuads;
        }

        public IReadOnlyList<GaussPoint> GenerateIntegrationPoints(IXFiniteElement element)
        {
            var points = new List<GaussPoint>();
            double length = 2.0 / quadsPerAxis;
            for (int i = 0; i < quadsPerAxis; ++i)
            {
                for (int j = 0; j < quadsPerAxis; ++j)
                {
                    // The borders of the subrectangle
                    double xiMin = -1.0 + length * i;
                    double xiMax = -1.0 + length * (i+1);
                    double etaMin = -1.0 + length * j;
                    double etaMax = -1.0 + length * (j + 1);

                    foreach(var subgridPoint in gaussQuadrature.IntegrationPoints)
                    {
                        // Transformation from the system of the subrectangle to the natural system of the element
                        double naturalXi = subgridPoint.Coordinates[0] * (xiMax - xiMin) / 2.0 + (xiMin + xiMax) / 2.0;
                        double naturalEta = subgridPoint.Coordinates[1] * (etaMax - etaMin) / 2.0 + (etaMin + etaMax) / 2.0;
                        double naturalWeight = subgridPoint.Weight * (xiMax - xiMin) / 2.0 * (etaMax - etaMin) / 2.0;
                        points.Add(new GaussPoint(new double[] { naturalXi, naturalEta }, naturalWeight));
                    }
                }
            }
            return points;
        }
    }
}
