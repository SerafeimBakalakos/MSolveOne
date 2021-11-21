using System.Collections.Generic;

using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Interpolation.GaussPointExtrapolation;
using MGroup.XFEM.Materials;

namespace MGroup.XFEM.Elements
{
	public class XCrackElementFactory3D : IXElementFactory<IXCrackElement>
	{
		private static readonly IReadOnlyDictionary<CellType, IElementGeometry> elementGeometries;
		private static readonly IReadOnlyDictionary<CellType, IGaussPointExtrapolation> extrapolations;
		private static readonly IReadOnlyDictionary<CellType, IQuadrature> stdIntegrationsForStiffness;
		//private static readonly IReadOnlyDictionary<CellType, IQuadrature> integrationsForMass;
		private static readonly IReadOnlyDictionary<CellType, IIsoparametricInterpolation> interpolations;

		//private readonly int integrationBoundaryOrder;
		private readonly CrackElementIntegrationStrategy integrationbulk;
		private readonly IFractureMaterialField material;

		static XCrackElementFactory3D()
		{
			// Mass integrations require as many Gauss points as there are nodes, in order for the consistent mass matrix to be
			// of full rank (and symmetric positive definite)

			// Collections' declarations
			var interpolations = new Dictionary<CellType, IIsoparametricInterpolation>();
			var stdIntegrationsForStiffness = new Dictionary<CellType, IQuadrature>();
			//var integrationsForMass = new Dictionary<CellType, IQuadrature2D>();
			var extrapolations = new Dictionary<CellType, IGaussPointExtrapolation>();
			var elementGeometries = new Dictionary<CellType, IElementGeometry>();

			// Hexa8
			interpolations.Add(CellType.Hexa8, InterpolationHexa8.UniqueInstance);
			stdIntegrationsForStiffness.Add(CellType.Hexa8, GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2));
			//integrationsForMass.Add(CellType.Hexa8, GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2));
			extrapolations.Add(CellType.Hexa8, ExtrapolationGaussLegendre2x2x2.UniqueInstance);
			elementGeometries.Add(CellType.Hexa8, new ElementHexa8Geometry());

			// Tet4
			interpolations.Add(CellType.Tet4, InterpolationTet4.UniqueInstance);
			stdIntegrationsForStiffness.Add(CellType.Tet4, TetrahedronQuadrature.Order2Points4);
			//integrationsForMass.Add(CellType.Tet4, TetrahedronQuadrature.Order2Points4);
			extrapolations.Add(CellType.Tet4, null);
			elementGeometries.Add(CellType.Tet4, new ElementTet4Geometry());

			// Static field assignments
			XCrackElementFactory3D.interpolations = interpolations;
			XCrackElementFactory3D.extrapolations = extrapolations;
			XCrackElementFactory3D.stdIntegrationsForStiffness = stdIntegrationsForStiffness;
			//XContinuumElement2DFactory.integrationsForMass = integrationsForMass;
			XCrackElementFactory3D.elementGeometries = elementGeometries;

		}

		public XCrackElementFactory3D(IFractureMaterialField commonMaterial,
			CrackElementIntegrationStrategy bulkIntegration/*, int integrationBoundaryOrder*/)
		{
			this.material = commonMaterial;
			this.integrationbulk = bulkIntegration;
			//this.integrationBoundaryOrder = integrationBoundaryOrder;
		}

		public bool UseStandardIntegrationForKss { get; set; } = false;

		public IXCrackElement CreateElement(int id, CellType cellType, IReadOnlyList<XNode> nodes)
		{
#if DEBUG
			interpolations[cellType].CheckElementNodes(nodes);
#endif
			return new XCrackElement3D(id, nodes, elementGeometries[cellType], material, interpolations[cellType],
				extrapolations[cellType], stdIntegrationsForStiffness[cellType], integrationbulk, UseStandardIntegrationForKss);
		}
	}
}
