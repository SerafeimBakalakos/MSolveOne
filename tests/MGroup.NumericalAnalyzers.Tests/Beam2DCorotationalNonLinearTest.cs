namespace MGroup.Analyzers.Tests
{
	using System.Collections.Generic;
	using MGroup.NumericalAnalyzers.NonLinear;
	using MGroup.FEM.Elements.SupportiveClasses;
	using MGroup.FEM.Entities;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.NumericalAnalyzers.Logging;
	using MGroup.MSolve.Solution;
	using Xunit;
	using MGroup.Solvers.Direct;
	using MGroup.NumericalAnalyzers;
	using MGroup.Constitutive.Structural;
	using MGroup.Constitutive.Structural.ContinuumElements;
	using MGroup.FEM.Structural.Elements;
	using MGroup.Solvers.AlgebraicModel;

	public class Beam2DCorotationalNonLinearTest
	{
		private const int subdomainID = 0;

		[Fact]
		private static void CantileverBeam2DCorotationalNonlinearTest()
		{
			double youngModulus = 21000.0;
			double poissonRatio = 0.3;
			double nodalLoad = 20000.0;
			double area = 91.04;
			double inertia = 8091.0;
			int nNodes = 3;
			int nElems = 2;
			int monitorNode = 3;

			// Node creation
			IList<Node> nodes = new List<Node>();
			Node node1 = new Node(id: 1, x: 0.0, y: 0.0);
			Node node2 = new Node(id: 2, x: 100.0, y: 0.0);
			Node node3 = new Node(id: 3, x: 200.0, y: 0.0);

			nodes.Add(node1);
			nodes.Add(node2);
			nodes.Add(node3);

			// Model creation
			var model = new Model();

			// Add a single subdomain to the model
			model.SubdomainsDictionary.Add(subdomainID, new Subdomain(subdomainID));

			// Add nodes to the nodes dictonary of the model
			for (int i = 0; i < nodes.Count; ++i)
			{
				model.NodesDictionary.Add(i + 1, nodes[i]);
			}

			// Constrain bottom nodes of the model
			model.NodesDictionary[1].Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
			model.NodesDictionary[1].Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
			model.NodesDictionary[1].Constraints.Add(new Constraint() { DOF = StructuralDof.RotationZ, Amount = 0.0 });

			// Generate elements of the structure
			int iNode = 1;
			for (int iElem = 0; iElem < nElems; iElem++)
			{
				// element nodes
				IList<Node> elementNodes = new List<Node>();
				elementNodes.Add(model.NodesDictionary[iNode]);
				elementNodes.Add(model.NodesDictionary[iNode + 1]);

				// Create new Beam3D section and element
				var beamSection = new BeamSection2D(area, inertia);

				// Create elements
				var element = new Element()
				{
					ID = iElem + 1,
					ElementType = new Beam2DCorotational(elementNodes, youngModulus, poissonRatio, 7.85, beamSection)
				};

				// Add nodes to the created element
				element.AddNode(model.NodesDictionary[iNode]);
				element.AddNode(model.NodesDictionary[iNode + 1]);

				var a = element.ElementType.StiffnessMatrix(element);

				// Add beam element to the element and subdomains dictionary of the model
				model.ElementsDictionary.Add(element.ID, element);
				model.SubdomainsDictionary[subdomainID].Elements.Add(element);
				iNode++;
			}

			// Add nodal load values at the top nodes of the model
			model.Loads.Add(new Load() { Amount = nodalLoad, Node = model.NodesDictionary[monitorNode], DOF = StructuralDof.TranslationY });

			// Choose linear equation system solver
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);

			// If we reorder, the expected displacements might correspond to different dofs
			//solverFactory.DofOrderer = new DofOrderer(new SimpleDofOrderingStrategy(), AmdReordering.CreateWithSuiteSparseAmd());
			ISolver solver = solverFactory.BuildSolver(algebraicModel);

			// Choose the provider of the problem -> here a structural problem
			var provider = new ProblemStructural(model, algebraicModel, solver);

			// Choose child analyzer -> Child: NewtonRaphsonNonLinearAnalyzer
			int increments = 10;
			var childAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, provider, increments);
			LoadControlAnalyzer childAnalyzer = childAnalyzerBuilder.Build();

			// Choose parent analyzer -> Parent: Static
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, provider, childAnalyzer);

			// Request output
			var watchDofs = new List<(INode node, IDofType dof)>();
			watchDofs.Add((model.NodesDictionary[3], StructuralDof.TranslationY));
			childAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check output
			DOFSLog log = (DOFSLog)childAnalyzer.Logs[0];
			Assert.Equal(146.5587362562, log.DOFValues[watchDofs[0].node, watchDofs[0].dof], 3);
		}

		[Fact]
		public void CantileverBeam2DCorotationalDisplacementControlTest()
		{
			double youngModulus = 21000.0;
			double poissonRatio = 0.3;
			double nodalDisplacement = 146.0;
			double area = 91.04;
			double inertia = 8091.0;
			int nNodes = 3;
			int nElems = 2;
			int monitorNode = 3;

			// Node creation
			IList<Node> nodes = new List<Node>();
			Node node1 = new Node(id: 1, x: 0.0, y: 0.0);
			Node node2 = new Node(id: 2, x: 100.0, y: 0.0);
			Node node3 = new Node(id: 3, x: 200.0, y: 0.0);

			nodes.Add(node1);
			nodes.Add(node2);
			nodes.Add(node3);

			// Model creation
			Model model = new Model();

			// Add a single subdomain to the model
			model.SubdomainsDictionary.Add(subdomainID, new Subdomain(subdomainID));

			// Add nodes to the nodes dictonary of the model
			for (int i = 0; i < nodes.Count; ++i)
			{
				model.NodesDictionary.Add(i + 1, nodes[i]);
			}

			// Constrain bottom nodes of the model
			model.NodesDictionary[1].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
			model.NodesDictionary[1].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
			model.NodesDictionary[1].Constraints.Add(new Constraint { DOF = StructuralDof.RotationZ });

			// Applied displacement
			model.NodesDictionary[3].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY, Amount = nodalDisplacement });

			// Generate elements of the structure
			int iNode = 1;
			for (int iElem = 0; iElem < nElems; iElem++)
			{
				// element nodes
				IList<Node> elementNodes = new List<Node>();
				elementNodes.Add(model.NodesDictionary[iNode]);
				elementNodes.Add(model.NodesDictionary[iNode + 1]);

				// Create new Beam3D section and element
				var beamSection = new BeamSection2D(area, inertia);

				// Create elements
				var element = new Element()
				{
					ID = iElem + 1,
					ElementType = new Beam2DCorotational(elementNodes, youngModulus, poissonRatio, 7.85, beamSection)
				};

				// Add nodes to the created element
				element.AddNode(model.NodesDictionary[iNode]);
				element.AddNode(model.NodesDictionary[iNode + 1]);

				var a = element.ElementType.StiffnessMatrix(element);

				// Add beam element to the element and subdomains dictionary of the model
				model.ElementsDictionary.Add(element.ID, element);
				model.SubdomainsDictionary[subdomainID].Elements.Add(element);
				iNode++;
			}

			// Choose linear equation system solver
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			ISolver solver = solverFactory.BuildSolver(algebraicModel);

			// Choose the provider of the problem -> here a structural problem
			var provider = new ProblemStructural(model, algebraicModel, solver);

			// Choose child analyzer -> Child: NewtonRaphsonNonLinearAnalyzer
			int numIncrements = 10;
			var childAnalyzerBuilder = new DisplacementControlAnalyzer.Builder(model, algebraicModel, solver, provider, numIncrements);
			var childAnalyzer = childAnalyzerBuilder.Build();

			// Choose parent analyzer -> Parent: Static
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, provider, childAnalyzer);

			// Request output
			var watchDofs = new List<(INode node, IDofType dof)>();
			watchDofs.Add((model.NodesDictionary[3], StructuralDof.TranslationX));
			childAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check output
			DOFSLog log = (DOFSLog)childAnalyzer.Logs[0]; 
			Assert.Equal(-72.090605787610343, log.DOFValues[watchDofs[0].node, watchDofs[0].dof], 7);
		}
	}
}
