using System.Collections.Generic;
using MGroup.FEM.Entities;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Discretization.Loads;
using Xunit;

namespace MGroup.FEM.Tests.IntegrationTests
{
	using Constitutive.Structural;
	using Constitutive.Structural.ContinuumElements;
	using ISAAR.MSolve.FEM.Elements;
	using ISAAR.MSolve.FEM.Interpolation;
	using MSolve.Constitutive;
	using MSolve.Solution;
	using NumericalAnalyzers;
	using NumericalAnalyzers.Logging;
	using NumericalAnalyzers.NonLinear;
	using Solvers.Direct;
	using Structural.Elements;

	public static class Hexa8NonLinearCantileverDefGrad
	{
		private const int subdomainID = 0;

		private static bool AreDisplacementsSame(IReadOnlyList<double[]> expectedDisplacements,
			TotalDisplacementsPerIterationLog computedDisplacements)
		{
			var comparer = new ValueComparer(1E-13);
			for (int iter = 0; iter < expectedDisplacements.Count; ++iter)
			{
				for (int d = 0; d < expectedDisplacements[iter].Length; ++d)
				{
					double expected = expectedDisplacements[iter][d];
					(INode node, IDofType dof) = computedDisplacements.WatchDofs[d];
					double computed = computedDisplacements.GetTotalDisplacement(iter, node, dof);
					if (!comparer.AreEqual(expected, computed))
					{
						return false;
					}
				}
			}
			return true;
		}

		[Fact]
		private static void RunTest()
		{
			IReadOnlyList<double[]> expectedDisplacements = GetExpectedDisplacements();
			TotalDisplacementsPerIterationLog computedDisplacements = SolveModel();
			Assert.True(AreDisplacementsSame(expectedDisplacements, computedDisplacements));
		}

		private static IReadOnlyList<double[]> GetExpectedDisplacements()
		{
			var expectedDisplacements = new double[11][]; //TODO: this should be 11 EINAI ARRAY APO DICTIONARIES

			expectedDisplacements[0] = new double[]  {
				 0.039075524153873623, -0.032541895181220408 -0.057387148941853101,  -0.071994381984550326,  -0.077053554770404833
			};

			expectedDisplacements[0] = new double[]  {
	3.907552415387362300e-02 , -3.254189518122040800e-02 , -5.738714894185310100e-02 , -7.199438198455032600e-02 , -7.705355477040483300e-02 };
			expectedDisplacements[1] = new double[]  {
	4.061313406968563400e-02 , -3.418876666892714500e-02 , -6.682708262609965400e-02 , -9.647418428408424700e-02 , -1.214556593711370000e-01 };
			expectedDisplacements[2] = new double[]  {
	4.036171804663909300e-02 , -3.396515033613205900e-02 , -6.665084050819490600e-02 , -9.713633946904017000e-02 , -1.236631490430697600e-01 };
			expectedDisplacements[3] = new double[]  {
	4.032905162001462800e-02 , -3.393260905426281900e-02 , -6.657423779424630200e-02 , -9.701032579889114200e-02 , -1.234941821043235900e-01 };
			expectedDisplacements[4] = new double[]  {
	4.032900093364350700e-02 , -3.393255831972321500e-02 , -6.657411965268195100e-02 , -9.701012513482368300e-02 , -1.234939001150344400e-01 };
			expectedDisplacements[5] = new double[]  {
	8.095088461395548400e-02 , -6.826589092291023000e-02 , -1.393261307096994000e-01 , -2.129883579558797000e-01 , -2.840192458274605800e-01 };
			expectedDisplacements[6] = new double[]  {
	8.179065808895391600e-02 , -6.914910025670165100e-02 , -1.449912527358244700e-01 , -2.283048858573358000e-01 , -3.126785624370127000e-01 };
			expectedDisplacements[7] = new double[]  {
	8.008398180684392400e-02 , -6.747544383562544000e-02 , -1.408463169597064000e-01 , -2.210877012127209200e-01 , -3.022981704019522300e-01 };
			expectedDisplacements[8] = new double[]  {
	7.976397887674688300e-02 , -6.715673915988762400e-02 , -1.400151566610138300e-01 , -2.195056794855129700e-01 , -2.998365539162924900e-01 };
			expectedDisplacements[9] = new double[]  {
	7.975945236918889600e-02 , -6.715223199537226400e-02 , -1.400036710136937400e-01 , -2.194845023343510200e-01 , -2.998046100841828000e-01 };
			expectedDisplacements[10] = new double[]  {
	7.975944951878896600e-02 , -6.715222916021290600e-02 , -1.400036636464831200e-01 , -2.194844883932760600e-01 , -2.998045884933974200e-01 };


			return expectedDisplacements;
		}

		private static TotalDisplacementsPerIterationLog SolveModel()
		{
			//VectorExtensions.AssignTotalAffinityCount();
			Model model = new Model();
			model.SubdomainsDictionary.Add(subdomainID, new Subdomain(subdomainID));

			BuildCantileverModel(model, 850);

			//model.ConnectDataStructures();

			//var linearSystems = new Dictionary<int, ILinearSystem>(); //I think this should be done automatically 
			//linearSystems[subdomainID] = new SkylineLinearSystem(subdomainID, model.Subdomains[0].Forces);

			//ProblemStructural provider = new ProblemStructural(model, linearSystems);

			//var solver = new SolverSkyline(linearSystems[subdomainID]);
			//var linearSystemsArray = new[] { linearSystems[subdomainID] };
			//var subdomainUpdaters = new[] { new NonLinearSubdomainUpdater(model.Subdomains[0]) };
			//var subdomainMappers = new[] { new SubdomainGlobalMapping(model.Subdomains[0]) };

			//var increments = 2;
			//var childAnalyzer = new NewtonRaphsonNonLinearAnalyzer(solver, linearSystemsArray, subdomainUpdaters, subdomainMappers, provider, increments, model.TotalDOFs);

			// Solver
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			ISolver solver = solverFactory.BuildSolver(algebraicModel);

			// Problem type
			var provider = new ProblemStructural(model, algebraicModel, solver);

			//var solver = new SolverSkyline(linearSystems[subdomainID]);
			//var linearSystemsArray = new[] { linearSystems[subdomainID] };

			//var subdomainUpdaters = new[] { new NonLinearSubdomainUpdater(model.Subdomains[0]) };
			//var subdomainMappers = new[] { new SubdomainGlobalMapping(model.Subdomains[0]) };

			var increments = 2;
			var childAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, provider, increments);
			childAnalyzerBuilder.ResidualTolerance = 1E-8;
			childAnalyzerBuilder.MaxIterationsPerIncrement = 100;
			childAnalyzerBuilder.NumIterationsForMatrixRebuild = 1;
			//childAnalyzerBuilder.SubdomainUpdaters = new[] { new NonLinearSubdomainUpdater(model.SubdomainsDictionary[subdomainID]) }; // This is the default
			LoadControlAnalyzer childAnalyzer = childAnalyzerBuilder.Build();
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, provider, childAnalyzer);

			var watchDofs = new List<(INode node, IDofType dof)>();
			watchDofs.Add((model.NodesDictionary[5], StructuralDof.TranslationX));
			watchDofs.Add((model.NodesDictionary[8], StructuralDof.TranslationZ));
			watchDofs.Add((model.NodesDictionary[12], StructuralDof.TranslationZ));
			watchDofs.Add((model.NodesDictionary[16], StructuralDof.TranslationZ));
			watchDofs.Add((model.NodesDictionary[20], StructuralDof.TranslationZ));
			var log1 = new TotalDisplacementsPerIterationLog(watchDofs, algebraicModel);
			childAnalyzer.TotalDisplacementsPerIterationLog = log1;


			//childAnalyzer.SetMaxIterations = 100;
			//childAnalyzer.SetIterationsForMatrixRebuild = 1;

			//StaticAnalyzer parentAnalyzer = new StaticAnalyzer(provider, childAnalyzer, linearSystems);

			//parentAnalyzer.BuildMatrices();
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();


			return log1;
		}

		private static void BuildCantileverModel(Model model, double load_value)
		{
			//xrhsimopoiithike to  ParadeigmataElegxwnBuilder.HexaCantileverBuilder(Model model, double load_value)
			// allagh tou element kai tou material

			//IContinuumMaterial3DTemp material1 = new IContinuumMaterial3DTemp()
			//{
			//    YoungModulus = 1353000,
			//    PoissonRatio = 0.3,
			//};


			//VonMisesMaterial3D material1 = new VonMisesMaterial3D(1353000, 0.30, 1353000, 0.15);
			IContinuumMaterial3DDefGrad material1 = new ElasticMaterial3DDefGrad() { PoissonRatio = 0.3, YoungModulus = 1353000 };

			double[,] nodeData = new double[,] { {-0.250000,-0.250000,-1.000000},
			{0.250000,-0.250000,-1.000000},
			{-0.250000,0.250000,-1.000000},
			{0.250000,0.250000,-1.000000},
			{-0.250000,-0.250000,-0.500000},
			{0.250000,-0.250000,-0.500000},
			{-0.250000,0.250000,-0.500000},
			{0.250000,0.250000,-0.500000},
			{-0.250000,-0.250000,0.000000},
			{0.250000,-0.250000,0.000000},
			{-0.250000,0.250000,0.000000},
			{0.250000,0.250000,0.000000},
			{-0.250000,-0.250000,0.500000},
			{0.250000,-0.250000,0.500000},
			{-0.250000,0.250000,0.500000},
			{0.250000,0.250000,0.500000},
			{-0.250000,-0.250000,1.000000},
			{0.250000,-0.250000,1.000000},
			{-0.250000,0.250000,1.000000},
			{0.250000,0.250000,1.000000}};

			int[,] elementData = new int[,] {{1,8,7,5,6,4,3,1,2},
			{2,12,11,9,10,8,7,5,6},
			{3,16,15,13,14,12,11,9,10},
			{4,20,19,17,18,16,15,13,14}, };

			// orismos shmeiwn
			for (int nNode = 0; nNode < nodeData.GetLength(0); nNode++)
			{
				model.NodesDictionary.Add(nNode + 1, new Node(id: nNode + 1, x: nodeData[nNode, 0], y: nodeData[nNode, 1], z: nodeData[nNode, 2]));

			}

			// orismos elements 
			Element e1;
			for (int nElement = 0; nElement < elementData.GetLength(0); nElement++)
			{
				List<Node> nodeSet = new List<Node>(8);
				for (int j = 0; j < 8; j++)
				{
					int nodeID = elementData[nElement, j + 1];
					nodeSet.Add((Node)model.NodesDictionary[nodeID]);
				}
				e1 = new Element()
				{
					ID = nElement + 1,
					ElementType  //new Hexa8NonLinearDefGrad(material1, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3)) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8                    
					= new ContinuumElement3DNonLinearDefGrad(nodeSet, material1, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3), InterpolationHexa8.UniqueInstance),

				};
				for (int j = 0; j < 8; j++)
				{
					e1.NodesDictionary.Add(elementData[nElement, j + 1], model.NodesDictionary[elementData[nElement, j + 1]]);
				}
				model.ElementsDictionary.Add(e1.ID, e1);
				model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
			}

			// constraint vashh opou z=-1
			for (int k = 1; k < 5; k++)
			{
				model.NodesDictionary[k].Constraints.Add(new Constraint()
				{
					Amount = 0,
					DOF = StructuralDof.TranslationX
				});
				model.NodesDictionary[k].Constraints.Add(new Constraint()
				{
					Amount = 0,
					DOF = StructuralDof.TranslationY
				});
				model.NodesDictionary[k].Constraints.Add(new Constraint()
				{
					Amount = 0,
					DOF = StructuralDof.TranslationZ
				});
			}

			// fortish korufhs
			Load load1;
			for (int k = 17; k < 21; k++)
			{
				load1 = new Load()
				{
					Node = model.NodesDictionary[k],
					DOF = StructuralDof.TranslationX,
					Amount = 1 * load_value
				};
				model.Loads.Add(load1);
			}
		}
	}

}
