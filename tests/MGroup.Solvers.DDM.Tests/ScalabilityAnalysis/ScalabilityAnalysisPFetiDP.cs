using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.IterativeMethods;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.Psm;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.ScalabilityAnalysis
{
	public class ScalabilityAnalysisPFetiDP : ScalabilityAnalysisBase
	{
		private static readonly IComputeEnvironment environment = new SequentialSharedEnvironment();

		//[Fact]
		public static void RunFullScalabilityAnalysisCantilever2D()
		{
			string outputDirectory = @"C:\Users\Serafeim\Desktop\PFETIDP\results\cantilever2D\";
			var scalabilityAnalysis = new ScalabilityAnalysisPFetiDP();
			scalabilityAnalysis.ModelBuilder = new CantilevelBeam2D();
			scalabilityAnalysis.EnableNativeDlls = false;
			scalabilityAnalysis.IterativeResidualTolerance = 1E-6;

			scalabilityAnalysis.RunParametricConstNumSubdomains(outputDirectory);
			//scalabilityAnalysis.RunParametricConstNumElements(outputDirectory);
			//scalabilityAnalysis.RunParametricConstSubdomainPerElementSize(outputDirectory);
		}

		//[Fact]
		//public static void RunFullScalabilityAnalysisCantilever3D()
		//{
		//	IComputeEnvironment environment = new SequentialSharedEnvironment();
		//	string outputDirectory = @"C:\Users\Serafeim\Desktop\PFETIDP\results\cantilever3D\";
		//	var scalabilityAnalysis = new ScalabilityAnalysisPFetiDP();
		//	scalabilityAnalysis.ModelBuilder = new CantilevelBeam3D();
		//	scalabilityAnalysis.EnableNativeDlls = true;
		//	scalabilityAnalysis.IterativeResidualTolerance = 1E-6;

		//	scalabilityAnalysis.RunParametricConstNumSubdomains(outputDirectory);
		//	scalabilityAnalysis.RunParametricConstNumElements(outputDirectory);
		//	scalabilityAnalysis.RunParametricConstSubdomainPerElementSize(outputDirectory);
		//}

		//[Fact]
		public static void RunFullScalabilityAnalysisRve2D()
		{
			IComputeEnvironment environment = new SequentialSharedEnvironment();
			string outputDirectory = @"C:\Users\Serafeim\Desktop\PFETIDP\results\rve2D\";
			var scalabilityAnalysis = new ScalabilityAnalysisPFetiDP();
			scalabilityAnalysis.ModelBuilder = new Rve2D();
			scalabilityAnalysis.EnableNativeDlls = true;
			scalabilityAnalysis.IterativeResidualTolerance = 1E-6;

			scalabilityAnalysis.RunParametricConstNumSubdomains(outputDirectory);
			scalabilityAnalysis.RunParametricConstNumElements(outputDirectory);
			scalabilityAnalysis.RunParametricConstSubdomainPerElementSize(outputDirectory);
		}

		//[Fact]
		//public static void RunFullScalabilityAnalysisRve3D()
		//{
		//  IComputeEnvironment environment = new SequentialSharedEnvironment();
		//	string outputDirectory = @"C:\Users\Serafeim\Desktop\PFETIDP\results\rve3D\";
		//	var scalabilityAnalysis = new ScalabilityAnalysisPFetiDP();
		//	scalabilityAnalysis.ModelBuilder = new Rve3D();
		//	scalabilityAnalysis.EnableNativeDlls = true;
		//	scalabilityAnalysis.IterativeResidualTolerance = 1E-6;

		//	scalabilityAnalysis.RunParametricConstNumSubdomains(outputDirectory);
		//	scalabilityAnalysis.RunParametricConstNumElements(outputDirectory);
		//	scalabilityAnalysis.RunParametricConstSubdomainPerElementSize(outputDirectory);
		//}

		public override (ISolver solver, IAlgebraicModel algebraicModel) CreateSolver(
			IModel model, ICornerDofSelection cornerDofs, ComputeNodeTopology nodeTopology)
		{
			environment.Initialize(nodeTopology);

			// Specify the format of matrices
			IPsmSubdomainMatrixManagerFactory<SymmetricCscMatrix> psmMatricesFactory;
			IFetiDPSubdomainMatrixManagerFactory<SymmetricCscMatrix> fetiDPMatricesFactory;
			IFetiDPCoarseProblemFactory fetiDPCoarseProblemFactory;
			if (EnableNativeDlls)
			{
				psmMatricesFactory = new PsmSubdomainMatrixManagerSymmetricSuiteSparse.Factory();
				fetiDPMatricesFactory = new FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory();
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricSuiteSparse();
				fetiDPCoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			}
			else
			{
				psmMatricesFactory = new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
				fetiDPMatricesFactory = new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory();
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
				fetiDPCoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			}

			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, psmMatricesFactory, cornerDofs, fetiDPMatricesFactory);
			solverFactory.CoarseProblemFactory = fetiDPCoarseProblemFactory;

			var interfaceProblemPcgBuilder = new PcgAlgorithm.Builder();
			interfaceProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
			interfaceProblemPcgBuilder.ResidualTolerance = 1E-10;
			solverFactory.InterfaceProblemSolver = interfaceProblemPcgBuilder.Build();
			solverFactory.IsHomogeneousProblem = true;

			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel);

			return (solver, algebraicModel);
		}
	}
}
