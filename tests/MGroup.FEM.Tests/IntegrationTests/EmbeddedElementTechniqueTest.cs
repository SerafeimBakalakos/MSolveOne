using System.Collections.Generic;
using System.Linq;

using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.ContinuumElements;
using MGroup.FEM.Elements.SupportiveClasses;
using MGroup.FEM.Entities;
using MGroup.FEM.Structural.Elements;
using MGroup.FEM.Structural.Embedding;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.NonLinear;
using MGroup.Solvers.Direct;

using Xunit;

namespace MGroup.FEM.Tests.IntegrationTests
{
	public class EmbeddedElementTechniqueTest
	{
		private const int subdomainID = 0;

		[Fact]
		public void EmbeddedElementTechniqueExample()
		{
			var model = new Model();
			model.SubdomainsDictionary.Add(subdomainID, new Subdomain(subdomainID));

			// Choose model
			EmbeddedExamplesBuilder.ExampleWithEmbedded(model);

			// Choose linear equation system solver
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			SkylineSolver solver = solverFactory.BuildSolver(algebraicModel);

			// Choose the provider of the problem -> here a structural problem
			var provider = new ProblemStructural(model, algebraicModel, solver);

			// Choose child analyzer -> Child: NewtonRaphsonNonLinearAnalyzer
			int increments = 10;
			var loadControlBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, provider, increments);
			LoadControlAnalyzer childAnalyzer = loadControlBuilder.Build();

			// Choose parent analyzer -> Parent: Static
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, provider, childAnalyzer);

			// Request output
			var watchDofs = new List<(INode node, IDofType dof)>();
			watchDofs.Add((model.NodesDictionary[8], StructuralDof.TranslationZ));
			childAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);
			//childAnalyzer.LogFactories[1] = new LinearAnalyzerLogFactory(new int[] {
			//    model.GlobalDofOrdering.GlobalFreeDofs[model.NodesDictionary[5], DOFType.X],
			//    model.GlobalDofOrdering.GlobalFreeDofs[model.NodesDictionary[5], DOFType.Y],
			//    model.GlobalDofOrdering.GlobalFreeDofs[model.NodesDictionary[5], DOFType.Z]});

			// Run
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check output
			DOFSLog log = (DOFSLog)childAnalyzer.Logs[0];
			var computedValue = log.DOFValues[watchDofs[0].node, watchDofs[0].dof];
			Assert.Equal(11.584726466617692, computedValue, 3);
		}

		public static class EmbeddedExamplesBuilder
		{
			public static void HostElementsBuilder(Model model)
			{
				// Nodes Geometry
				model.NodesDictionary.Add(1, new Node(id: 1, x: 10.00, y: 2.50, z: 2.50));
				model.NodesDictionary.Add(2, new Node(id: 2, x: 0.00, y: 2.50, z: 2.50));
				model.NodesDictionary.Add(3, new Node(id: 3, x: 0.00, y: -2.50, z: 2.50));
				model.NodesDictionary.Add(4, new Node(id: 4, x: 10.00, y: -2.50, z: 2.50));
				model.NodesDictionary.Add(5, new Node(id: 5, x: 10.00, y: 2.50, z: -2.50));
				model.NodesDictionary.Add(6, new Node(id: 6, x: 0.00, y: 2.50, z: -2.50));
				model.NodesDictionary.Add(7, new Node(id: 7, x: 0.00, y: -2.50, z: -2.50));
				model.NodesDictionary.Add(8, new Node(id: 8, x: 10.00, y: -2.50, z: -2.50));

				// Boundary Conditions
				model.NodesDictionary[2].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
				model.NodesDictionary[2].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
				model.NodesDictionary[2].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });
				model.NodesDictionary[3].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
				model.NodesDictionary[3].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
				model.NodesDictionary[3].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });
				model.NodesDictionary[6].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
				model.NodesDictionary[6].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
				model.NodesDictionary[6].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });
				model.NodesDictionary[7].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
				model.NodesDictionary[7].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
				model.NodesDictionary[7].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });

				// Create Material
				var solidMaterial = new ElasticMaterial3D()
				{
					YoungModulus = 3.76,
					PoissonRatio = 0.3779,
				};

				DynamicMaterial DynamicMaterial = new DynamicMaterial(1, 0, 0);
				var factory = new ContinuumElement3DFactory(solidMaterial, DynamicMaterial);
				// Hexa8NL element definition
				List<Node> nodeSet = new List<Node>(8);
				for (int j = 1; j <  9; j++)
				{
					
					nodeSet.Add((Node)model.NodesDictionary[j]);
				}
				var hexa8NLelement = new Element()
				{
					ID = 1,
					ElementType=// factory.CreateNonLinearElement(CellType.Hexa8, nodeSet, solidMaterial, DynamicMaterial)
					 new Hexa8NonLinear(solidMaterial, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3))
					// = new ContinummElement3DNonLinear(nodeSet, solidMaterial, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3), ISAAR.MSolve.FEM.Interpolation.InterpolationHexa8.UniqueInstance)
				};

				// Add nodes to the created element
				hexa8NLelement.AddNode(model.NodesDictionary[1]);
				hexa8NLelement.AddNode(model.NodesDictionary[2]);
				hexa8NLelement.AddNode(model.NodesDictionary[3]);
				hexa8NLelement.AddNode(model.NodesDictionary[4]);
				hexa8NLelement.AddNode(model.NodesDictionary[5]);
				hexa8NLelement.AddNode(model.NodesDictionary[6]);
				hexa8NLelement.AddNode(model.NodesDictionary[7]);
				hexa8NLelement.AddNode(model.NodesDictionary[8]);

				// Add Hexa element to the element and subdomains dictionary of the model
				model.ElementsDictionary.Add(hexa8NLelement.ID, hexa8NLelement);
				model.SubdomainsDictionary[subdomainID].Elements.Add(hexa8NLelement);

				// Add nodal load values at the top nodes of the model
				model.Loads.Add(new Load() { Amount = 25, Node = model.NodesDictionary[1], DOF = StructuralDof.TranslationZ });
				model.Loads.Add(new Load() { Amount = 25, Node = model.NodesDictionary[4], DOF = StructuralDof.TranslationZ });
				model.Loads.Add(new Load() { Amount = 25, Node = model.NodesDictionary[5], DOF = StructuralDof.TranslationZ });
				model.Loads.Add(new Load() { Amount = 25, Node = model.NodesDictionary[8], DOF = StructuralDof.TranslationZ });
			}

			public static void EmbeddedElementsBuilder(Model model)
			{
				// define mechanical properties
				double youngModulus = 1.0;
				double shearModulus = 1.0;
				double poissonRatio = (youngModulus / (2 * shearModulus)) - 1;
				double area = 1776.65;  // CNT(20,20)-LinearEBE-TBT-L = 10nm
				double inertiaY = 1058.55;
				double inertiaZ = 1058.55;
				double torsionalInertia = 496.38;
				double effectiveAreaY = area;
				double effectiveAreaZ = area;

				// Geometry
				model.NodesDictionary.Add(9, new Node(id: 9, x: 0.00, y: 0.00, z: 0.00));
				model.NodesDictionary.Add(10, new Node(id: 10, x: 10.00, y: 0.00, z: 0.00));

				// Create new 3D material
				var beamMaterial = new ElasticMaterial3D()
				{
					YoungModulus = youngModulus,
					PoissonRatio = poissonRatio,
				};

				// Create new Beam3D section and element
				var beamSection = new BeamSection3D(area, inertiaY, inertiaZ, torsionalInertia, effectiveAreaY, effectiveAreaZ);
				// element nodes
				var elementNodes = new List<Node>();
				elementNodes.Add(model.NodesDictionary[9]);
				elementNodes.Add(model.NodesDictionary[10]);
				var beam = new Beam3DCorotationalQuaternion(elementNodes, youngModulus, poissonRatio, 7.85, beamSection);
				var beamElement = new Element { ID = 2, ElementType = beam };

				beamElement.NodesDictionary.Add(9, model.NodesDictionary[9]);
				beamElement.NodesDictionary.Add(10, model.NodesDictionary[10]);

				model.ElementsDictionary.Add(beamElement.ID, beamElement);
				model.SubdomainsDictionary[subdomainID].Elements.Add(beamElement);
			}

			public static void ExampleWithEmbedded(Model model)
			{
				HostElementsBuilder(model);
				EmbeddedElementsBuilder(model);
				var embeddedGrouping = new EmbeddedGrouping(model, model.ElementsDictionary.Where(x => x.Key == 1).Select(kv => kv.Value), model.ElementsDictionary.Where(x => x.Key == 2).Select(kv => kv.Value), true);
			}
		}
	}
}
