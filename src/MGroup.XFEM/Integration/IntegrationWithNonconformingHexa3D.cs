using System.Collections.Generic;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;

namespace MGroup.XFEM.Integration
{
	public class IntegrationWithNonconformingHexa3D : IBulkIntegration
	{
		private readonly int hexaPerAxis;
		private readonly GaussLegendre3D  gaussQuadrature;

		public IntegrationWithNonconformingHexa3D(int hexaPerAxis) : 
			this(hexaPerAxis, GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2))
		{
		}

		public IntegrationWithNonconformingHexa3D(int hexaPerAxis, GaussLegendre3D quadratureInHexa)
		{
			this.hexaPerAxis = hexaPerAxis;
			this.gaussQuadrature = quadratureInHexa;
		}

		public IReadOnlyList<GaussPoint> GenerateIntegrationPoints(IXFiniteElement element)
		{
			var points = new List<GaussPoint>();
			double length = 2.0 / hexaPerAxis;
			for (int i = 0; i < hexaPerAxis; ++i)
			{
				// The borders of the subrectangle
				double xiMin = -1.0 + length * i;
				double xiMax = -1.0 + length * (i + 1);
				for (int j = 0; j < hexaPerAxis; ++j)
				{
					double etaMin = -1.0 + length * j;
					double etaMax = -1.0 + length * (j + 1);
					for (int k = 0; k < hexaPerAxis; ++k)
					{
						double zetaMin = -1.0 + length * k;
						double zetaMax = -1.0 + length * (k + 1);

						foreach (var subgridPoint in gaussQuadrature.IntegrationPoints)
						{
							// Transformation from the system of the subrectangle to the natural system of the element
							double naturalXi = subgridPoint.Coordinates[0] * (xiMax - xiMin) / 2.0 + (xiMin + xiMax) / 2.0;
							double naturalEta = subgridPoint.Coordinates[1] * (etaMax - etaMin) / 2.0 + (etaMin + etaMax) / 2.0;
							double naturalZeta = subgridPoint.Coordinates[2] * (zetaMax - zetaMin) / 2.0 + (zetaMin + zetaMax) / 2.0;
							double naturalWeight = subgridPoint.Weight * (xiMax - xiMin) / 2.0 * (etaMax - etaMin) / 2.0;
							points.Add(new GaussPoint(new double[] { naturalXi, naturalEta, naturalZeta }, naturalWeight));
						}
					}
				}
			}
			return points;
		}
	}
}
