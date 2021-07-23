using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.PlanarElements;
using MGroup.Environments;
using MGroup.FEM.Entities;
using MGroup.FEM.Interfaces;
using MGroup.FEM.Structural.Elements;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Meshes.Structured;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.Partitioning;
using MGroup.Solvers.Direct;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.Results;

// Subdomains:
// /|                 |\
// /||-------|-------||\
// /||  (2)  |  (3)  ||\
// /||   E1  |   E0  ||\
// /||-------|-------||\
// /||  (0)  |  (1)  ||\
// /||   E1  |   E0  ||\
// /||-------|-------||\
// /|				  |\
namespace MGroup.Solvers.DDM.Tests.ExampleModels
{
	/// <summary>
	/// See Papagiannakis Bachelor thesis, pages 148-161
	/// </summary>
	public static class PapagiannakisModel_9_2
	{
		private const double E0 = 2.1E7, v = 0.3, thickness = 0.3;
		private const double load = 100;

		public static double[] MinCoords => new double[] { 0, 0 };

		public static double[] MaxCoords => new double[] { 3, 1.5 };

		public static int[] NumElements => new int[] { 20, 20 };

		public static int[] NumSubdomains => new int[] { 2, 2 };

		public static int[] NumClusters => new int[] { 1, 1 };

		public static int NumTotalDofs => 882;

		public static Model CreateSingleSubdomainModel(double stiffnessRatio)
		{
			var model = new Model();
			model.AllDofs.AddDof(StructuralDof.TranslationX);
			model.AllDofs.AddDof(StructuralDof.TranslationY);
			model.SubdomainsDictionary[0] = new Subdomain(0);

			var mesh = new UniformCartesianMesh2D.Builder(MinCoords, MaxCoords, NumElements).SetMajorAxis(0).BuildMesh();

			// Nodes
			foreach ((int id, double[] coords) in mesh.EnumerateNodes())
			{
				model.NodesDictionary[id] = new Node(id, coords[0], coords[1]);
			}

			// Materials
			var material0 = new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = E0, PoissonRatio = v };
			var material1 = new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = stiffnessRatio * E0, PoissonRatio = v };
			var dynamicProperties = new DynamicMaterial(1.0, 1.0, 1.0);

			// Elements
			var elemFactory0 = new ContinuumElement2DFactory(thickness, material0, dynamicProperties);
			var elemFactory1 = new ContinuumElement2DFactory(thickness, material1, dynamicProperties);
			double dx = (MaxCoords[0] - MinCoords[0]) / NumElements[0];
			double meshTol = 1E-6 * dx;
			double xSeparator = MinCoords[0] + 0.25 * (MaxCoords[0] - MinCoords[0]) + meshTol;
			foreach ((int elementID, int[] nodeIDs) in mesh.EnumerateElements())
			{
				Node[] nodes = nodeIDs.Select(n => model.NodesDictionary[n]).ToArray();
				IFiniteElement elementType;
				if (nodes.All(n => n.X < xSeparator))
				{
					elementType = elemFactory1.CreateElement(mesh.CellType, nodes);
				}
				else
				{
					elementType = elemFactory0.CreateElement(mesh.CellType, nodes);
				}
				var element = new Element() { ID = elementID, ElementType = elementType };
				foreach (var node in nodes) element.AddNode(node);
				model.ElementsDictionary[element.ID] = element;
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			// Boundary conditions
			Node[] leftNodes = model.NodesDictionary.Values.Where(n => n.X < MinCoords[0] + meshTol).ToArray();
			Node[] rightNodes = model.NodesDictionary.Values.Where(n => n.X > MaxCoords[0] - meshTol).ToArray();
			foreach (Node node in leftNodes.Union(rightNodes))
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
			}

			Node[] topNodes = model.NodesDictionary.Values.Where(n => n.Y > MaxCoords[1] - meshTol).ToArray();
			foreach (Node node in topNodes)
			{
				model.Loads.Add(new Load() { Node = node, DOF = StructuralDof.TranslationY, Amount = load / rightNodes.Length });
			}

			return model;
		}

		public static (Model model, ComputeNodeTopology topology) CreateMultiSubdomainModel(double stiffnessRatio)
		{
			var model = CreateSingleSubdomainModel(stiffnessRatio);
			var mesh = new UniformCartesianMesh2D.Builder(MinCoords, MaxCoords, NumElements).SetMajorAxis(0).BuildMesh();
			var partitioner = new UniformMeshPartitioner2D(mesh, NumSubdomains, NumClusters);
			partitioner.Partition(model);
			ModelUtilities.DecomposeIntoSubdomains(model, partitioner.NumSubdomainsTotal, partitioner.GetSubdomainOfElement);

			var topology = new ComputeNodeTopology();
			for (int s = 0; s < partitioner.NumSubdomainsTotal; ++s)
			{
				topology.AddNode(s, partitioner.GetNeighboringSubdomains(s), partitioner.GetClusterOfSubdomain(s));
			}

			return (model, topology);
		}

		public static ICornerDofSelection GetCornerDofs(IModel model)
		{
			var cornerNodes = new HashSet<int>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				INode[] subdomainCorners = CornerNodeUtilities.FindCornersOfRectangle2D(subdomain);
				foreach (INode node in subdomainCorners)
				{
					if (node.Subdomains.Count > 1) //TODO for some reason this does not work if > 2. One Krr is singular.
					{
						cornerNodes.Add(node.ID);
					}
				}
			}

			var cornerDofs = new UserDefinedCornerDofSelection();
			foreach (int node in cornerNodes)
			{
				cornerDofs.AddCornerNode(node);
			}
			return cornerDofs;
		}

		public static NodalResults SolveWithSkylineSolver(double stiffnessRatio)
		{
			Model model = CreateSingleSubdomainModel(stiffnessRatio);
			model.ConnectDataStructures();

			// Solver
			var solverFactory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			SkylineSolver solver = solverFactory.BuildSolver(algebraicModel);

			// Linear static analysis
			var problem = new ProblemStructural(model, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			NodalResults nodalDisplacements = algebraicModel.ExtractAllResults(solver.LinearSystem.Solution);
			Debug.Assert(nodalDisplacements.Data.EntryCount == NumTotalDofs);
			return nodalDisplacements;
		}
	}
}
