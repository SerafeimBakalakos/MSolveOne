using System.Collections.Generic;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Interpolation.GaussPointExtrapolation;
using MGroup.XFEM.Materials;
using MGroup.XFEM.ElementGeometry;

namespace MGroup.XFEM.Elements
{
    public class XThermalElement2DFactory
    {
        private static readonly IReadOnlyDictionary<CellType, IElementGeometry> elementGeometries;
        private static readonly IReadOnlyDictionary<CellType, IGaussPointExtrapolation> extrapolations;
        private static readonly IReadOnlyDictionary<CellType, IQuadrature> standardIntegrationsForConductivity;
        //private static readonly IReadOnlyDictionary<CellType, IQuadrature2D> integrationsForMass;
        private static readonly IReadOnlyDictionary<CellType, IIsoparametricInterpolation> interpolations;

        private readonly int boundaryIntegrationOrder;
        private readonly bool cohesiveInterfaces;
        private readonly IBulkIntegration integrationbulk;
        private readonly IThermalMaterialField material;
        private readonly double thickness;

        static XThermalElement2DFactory()
        {
            // Mass integrations require as many Gauss points as there are nodes, in order for the consistent mass matrix to be
            // of full rank (and symmetric positive definite)

            // Collections' declarations
            var interpolations = new Dictionary<CellType, IIsoparametricInterpolation>();
            var standardIntegrationsForConductivity = new Dictionary<CellType, IQuadrature>();
            //var integrationsForMass = new Dictionary<CellType, IQuadrature2D>();
            var extrapolations = new Dictionary<CellType, IGaussPointExtrapolation>();
            var elementGeometries = new Dictionary<CellType, IElementGeometry>();

            // Quad4
            interpolations.Add(CellType.Quad4, InterpolationQuad4.UniqueInstance);
            standardIntegrationsForConductivity.Add(CellType.Quad4, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
            //integrationsForMass.Add(CellType.Quad4, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
            extrapolations.Add(CellType.Quad4, ExtrapolationGaussLegendre2x2.UniqueInstance);
            elementGeometries.Add(CellType.Quad4, new ElementQuad4Geometry());

            // Quad8
            interpolations.Add(CellType.Quad8, InterpolationQuad8.UniqueInstance);
            standardIntegrationsForConductivity.Add(CellType.Quad8, GaussLegendre2D.GetQuadratureWithOrder(3, 3));
            //integrationsForMass.Add(CellType.Quad8, GaussLegendre2D.GetQuadratureWithOrder(3, 3));
            extrapolations.Add(CellType.Quad8, ExtrapolationGaussLegendre3x3.UniqueInstance);
            //elementGeometries.Add(CellType.Quad8, new ElementQuad8Geometry());

            // Quad9
            interpolations.Add(CellType.Quad9, InterpolationQuad9.UniqueInstance);
            standardIntegrationsForConductivity.Add(CellType.Quad9, GaussLegendre2D.GetQuadratureWithOrder(3, 3));
            //integrationsForMass.Add(CellType.Quad9, GaussLegendre2D.GetQuadratureWithOrder(3, 3));
            extrapolations.Add(CellType.Quad9, ExtrapolationGaussLegendre3x3.UniqueInstance);
            //elementGeometries.Add(CellType.Quad9, new ElementQuad9Geometry());

            // Tri3
            interpolations.Add(CellType.Tri3, InterpolationTri3.UniqueInstance);
            standardIntegrationsForConductivity.Add(CellType.Tri3, TriangleQuadratureSymmetricGaussian.Order1Point1);
            //integrationsForMass.Add(CellType.Tri3, TriangleQuadratureSymmetricGaussian.Order2Points3);
            extrapolations.Add(CellType.Tri3, ExtrapolationGaussTriangular1Point.UniqueInstance);
            elementGeometries.Add(CellType.Tri3, new ElementTri3Geometry());

            // Tri 6
            interpolations.Add(CellType.Tri6, InterpolationTri6.UniqueInstance);
            // see https://www.colorado.edu/engineering/CAS/courses.d/IFEM.d/IFEM.Ch24.d/IFEM.Ch24.pdf, p. 24-13, paragraph "options"
            standardIntegrationsForConductivity.Add(CellType.Tri6, TriangleQuadratureSymmetricGaussian.Order2Points3);
            //integrationsForMass.Add(CellType.Tri6, TriangleQuadratureSymmetricGaussian.Order4Points6);
            extrapolations.Add(CellType.Tri6, ExtrapolationGaussTriangular3Points.UniqueInstance);
            //elementGeometries.Add(CellType.Tri6, new ElementTri6Geometry());

            // Static field assignments
            XThermalElement2DFactory.interpolations = interpolations;
            XThermalElement2DFactory.extrapolations = extrapolations;
            XThermalElement2DFactory.standardIntegrationsForConductivity = standardIntegrationsForConductivity;
            //XContinuumElement2DFactory.integrationsForMass = integrationsForMass;
            XThermalElement2DFactory.elementGeometries = elementGeometries;

        }

        public XThermalElement2DFactory(IThermalMaterialField commonMaterial, double thickness,
            IBulkIntegration bulkIntegration, int boundaryIntegrationOrder, bool cohesiveInterfaces)
        {
            this.material = commonMaterial;
            this.thickness = thickness;
            this.integrationbulk = bulkIntegration;
            this.boundaryIntegrationOrder = boundaryIntegrationOrder;
            this.cohesiveInterfaces = cohesiveInterfaces;
        }

        public XThermalElement2D CreateElement(int id, CellType cellType, IReadOnlyList<XNode> nodes)
        {
#if DEBUG
            interpolations[cellType].CheckElementNodes(nodes);
#endif
            return new XThermalElement2D(id, nodes, thickness, elementGeometries[cellType], material, interpolations[cellType],
                extrapolations[cellType], standardIntegrationsForConductivity[cellType], integrationbulk, 
                boundaryIntegrationOrder, cohesiveInterfaces);
        }
    }
}
