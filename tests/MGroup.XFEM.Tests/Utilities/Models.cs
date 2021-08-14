using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;

namespace MGroup.XFEM.Tests.Utilities
{
	public static class Models
	{
		public static void AddNodesElements<TElement>(
			XModel<TElement> model, IStructuredMesh mesh, IXElementFactory<TElement> factory) 
			where TElement : class, IXFiniteElement
		{
			int subdomainID = model.Subdomains.First().Key;

			// Nodes
			foreach ((int nodeID, double[] coords) in mesh.EnumerateNodes())
			{
				model.Nodes[nodeID] = new XNode(nodeID, coords);
			}

			// Elements
			foreach ((int elementID, int[] connectivity) in mesh.EnumerateElements())
			{
				var nodes = new XNode[connectivity.Length];
				for (int n = 0; n < connectivity.Length; ++n)
				{
					nodes[n] = model.Nodes[connectivity[n]];
				}
				var element = factory.CreateElement(elementID, mesh.CellType, nodes);
				model.Elements[elementID] = element;
				model.Subdomains[subdomainID].Elements.Add(element);
			}
		}

		public static void ApplyBCsCantileverTension(XModel<IXMultiphaseElement> model)
		{
			// Boundary conditions
			double meshTol = 1E-7;

			// Left side: Ux=Uy=Uz=0
			double minX = model.Nodes.Values.Select(n => n.X).Min();
			foreach (var node in model.Nodes.Values.Where(n => Math.Abs(n.X - minX) <= meshTol))
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
				if (model.Dimension == 3)
				{
					node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });
				}
			}

			// Right side: Fx = 100
			double maxX = model.Nodes.Values.Select(n => n.X).Max();
			XNode[] rightSideNodes = model.Nodes.Values.Where(n => Math.Abs(n.X - maxX) <= meshTol).ToArray();
			double load = 1.0 / rightSideNodes.Length;
			foreach (var node in rightSideNodes)
			{
				model.NodalLoads.Add(new Load() { Node = node, DOF = StructuralDof.TranslationX, Amount = load });
			}
		}

		public static void ApplyBCsTemperatureDiffAlongX(IXModel model, double temperatureAtMinX, double temperatureAtMaxX)
		{
			// Boundary conditions
			double meshTol = 1E-7;

			// Left side
			double minX = model.Nodes.Values.Select(n => n.X).Min();
			foreach (var node in model.Nodes.Values.Where(n => Math.Abs(n.X - minX) <= meshTol))
			{
				node.Constraints.Add(new Constraint() { DOF = ThermalDof.Temperature, Amount = temperatureAtMinX });
			}

			// Right side
			double maxX = model.Nodes.Values.Select(n => n.X).Max();
			foreach (var node in model.Nodes.Values.Where(n => Math.Abs(n.X - maxX) <= meshTol))
			{
				node.Constraints.Add(new Constraint() { DOF = ThermalDof.Temperature, Amount = temperatureAtMaxX });
			}
		}

		public static XModel<IXMultiphaseElement> CreateQuad4Model(double[] minCoords, double[] maxCoords, double thickness,
			int[] numElements, bool cartesianMesh, int bulkIntegrationOrder, int boundaryIntegrationOrder, 
			IThermalMaterialField materialField, bool cohesiveInterfaces)
		{
			var model = new XModel<IXMultiphaseElement>(2);
			model.Subdomains[0] = new XSubdomain<IXMultiphaseElement>(0);

			// Nodes
			IStructuredMesh mesh;
			if (cartesianMesh)
			{
				mesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			}
			else
			{
				int[] numNodes = { numElements[0] + 1, numElements[1] + 1 };
				mesh = new UniformSimplicialMesh2D.Builder(minCoords, maxCoords, numNodes).BuildMesh();
			}
			foreach ((int nodeID, double[] coords) in mesh.EnumerateNodes())
			{
				model.Nodes[nodeID] = new XNode(nodeID, coords);
			}

			// Integration
			var stdQuadrature = GaussLegendre2D.GetQuadratureWithOrder(bulkIntegrationOrder, bulkIntegrationOrder);
			var subcellQuadrature = TriangleQuadratureSymmetricGaussian.Order2Points3;
			var integrationBulk = new IntegrationWithConformingSubtriangles2D(subcellQuadrature);

			// Elements
			var elemFactory = new XThermalElement2DFactory(
				materialField, thickness, integrationBulk, boundaryIntegrationOrder, cohesiveInterfaces);
			foreach ((int elementID, int[] connectivity) in mesh.EnumerateElements())
			{
				var nodes = new XNode[connectivity.Length];
				for (int n = 0; n < connectivity.Length; ++n)
				{
					nodes[n] = model.Nodes[connectivity[n]];
				}
				var element = elemFactory.CreateElement(elementID, mesh.CellType, nodes);
				model.Elements[elementID] = element;
				model.Subdomains[0].Elements.Add(element);
			}
		   
			// Boundary conditions
			double meshTol = 1E-7;

			// Left side: T = +100
			double minX = model.Nodes.Values.Select(n => n.X).Min();
			foreach (var node in model.Nodes.Values.Where(n => Math.Abs(n.X - minX) <= meshTol))
			{
				node.Constraints.Add(new Constraint() { DOF = ThermalDof.Temperature, Amount = +100 });
			}

			// Right side: T = 100
			double maxX = model.Nodes.Values.Select(n => n.X).Max();
			foreach (var node in model.Nodes.Values.Where(n => Math.Abs(n.X - maxX) <= meshTol))
			{
				node.Constraints.Add(new Constraint() { DOF = ThermalDof.Temperature, Amount = -100 });
			}

			model.ConnectDataStructures();
			return model;
		}

		public static XModel<IXMultiphaseElement> CreateQuad4Model(double[] minCoords, double[] maxCoords, double thickness,
			int[] numElements, int bulkIntegrationOrder, int boundaryIntegrationOrder, IStructuralMaterialField materialField,
			bool cohesiveInterfaces)
		{
			var model = new XModel<IXMultiphaseElement>(2);
			model.Subdomains[0] = new XSubdomain<IXMultiphaseElement>(0);

			IStructuredMesh mesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements).BuildMesh();

			foreach ((int nodeID, double[] coords) in mesh.EnumerateNodes())
			{
				model.Nodes[nodeID] = new XNode(nodeID, coords);
			}

			// Integration
			//var stdQuadrature = GaussLegendre2D.GetQuadratureWithOrder(bulkIntegrationOrder, bulkIntegrationOrder);
			var subcellQuadrature = TriangleQuadratureSymmetricGaussian.Order2Points3;
			var integrationBulk = new IntegrationWithConformingSubtriangles2D(subcellQuadrature);

			// Elements
			var elemFactory = new XMultiphaseStructuralElementFactory2D(
				materialField, thickness, integrationBulk, boundaryIntegrationOrder, cohesiveInterfaces);
			foreach ((int elementID, int[] connectivity) in mesh.EnumerateElements())
			{
				var nodes = new XNode[connectivity.Length];
				for (int n = 0; n < connectivity.Length; ++n)
				{
					nodes[n] = model.Nodes[connectivity[n]];
				}
				var element = elemFactory.CreateElement(elementID, mesh.CellType, nodes);
				model.Elements[elementID] = element;
				model.Subdomains[0].Elements.Add(element);
			}

			return model;
		}

		public static XModel<IXMultiphaseElement> CreateHexa8Model(double[] minCoords, double[] maxCoords,int[] numElements,
			bool cartesianMesh, int bulkIntegrationOrder, int boundaryIntegrationOrder, IThermalMaterialField materialField, 
			bool cohesiveInterfaces)
		{
			var model = new XModel<IXMultiphaseElement>(3);
			model.Subdomains[0] = new XSubdomain<IXMultiphaseElement>(0);

			IStructuredMesh mesh;
			if (cartesianMesh)
			{
				mesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			}
			else
			{
				int[] numNodes = { numElements[0] + 1, numElements[1] + 1, numElements[2] + 1 };
				mesh = new UniformSimplicialMesh3D.Builder(minCoords, maxCoords, numNodes).BuildMesh();
			}

			foreach ((int nodeID, double[] coords) in mesh.EnumerateNodes())
			{
				model.Nodes[nodeID] = new XNode(nodeID, coords);
			}

			// Integration
			var subcellQuadrature = TetrahedronQuadrature.Order2Points4;
			var integrationBulk = new IntegrationWithConformingSubtetrahedra3D(subcellQuadrature);

			// Elements
			var elemFactory = new XThermalElement3DFactory(
				materialField, integrationBulk, boundaryIntegrationOrder, cohesiveInterfaces);
			foreach ((int elementID, int[] connectivity) in mesh.EnumerateElements())
			{
				var nodes = new XNode[connectivity.Length];
				for (int n = 0; n < connectivity.Length; ++n)
				{
					nodes[n] = model.Nodes[connectivity[n]];
				}
				var element = elemFactory.CreateElement(elementID, mesh.CellType, nodes);
				model.Elements[elementID] = element;
				model.Subdomains[0].Elements.Add(element);
			}

			return model;
		}

		public static XModel<IXMultiphaseElement> CreateHexa8Model(double[] minCoords, double[] maxCoords,
			int[] numElements, int bulkIntegrationOrder, int boundaryIntegrationOrder, IStructuralMaterialField materialField,
			bool cohesiveInterfaces)
		{
			var model = new XModel<IXMultiphaseElement>(3);
			model.Subdomains[0] = new XSubdomain<IXMultiphaseElement>(0);

			// Nodes
			IStructuredMesh mesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			foreach ((int nodeID, double[] coords) in mesh.EnumerateNodes())
			{
				model.Nodes[nodeID] = new XNode(nodeID, coords);
			}

			// Integration
			var subcellQuadrature = TetrahedronQuadrature.Order2Points4;
			var integrationBulk = new IntegrationWithConformingSubtetrahedra3D(subcellQuadrature);

			// Elements
			var elemFactory = new XMultiphaseStructuralElementFactory3D(
				materialField, integrationBulk, boundaryIntegrationOrder, cohesiveInterfaces);
			foreach ((int elementID, int[] connectivity) in mesh.EnumerateElements())
			{
				var nodes = new XNode[connectivity.Length];
				for (int n = 0; n < connectivity.Length; ++n)
				{
					nodes[n] = model.Nodes[connectivity[n]];
				}
				var element = elemFactory.CreateElement(elementID, mesh.CellType, nodes);
				model.Elements[elementID] = element;
				model.Subdomains[0].Elements.Add(element);
			}

			return model;
		}
	}
}
