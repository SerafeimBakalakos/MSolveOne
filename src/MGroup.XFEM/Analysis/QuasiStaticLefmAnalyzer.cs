using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Constitutive.Structural;
using MGroup.Environments.Mpi;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.LinearSystem;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Solvers;

namespace MGroup.XFEM.Analysis
{
	public class QuasiStaticLefmAnalyzer
	{
		private readonly XModel<IXCrackElement> model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ElementStructuralStiffnessProvider elementMatrixProvider;
		private readonly ISolver solver;
		private readonly int maxIterations;
		private readonly IPropagationTermination termination;
		private readonly bool reanalysis;

		public QuasiStaticLefmAnalyzer(XModel<IXCrackElement> model, IAlgebraicModel algebraicModel, ISolver solver, 
			int maxIterations, IPropagationTermination terminationCriterion, bool reanalysis = false)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
			this.maxIterations = maxIterations;
			this.termination = terminationCriterion;
			this.reanalysis = reanalysis;
			this.elementMatrixProvider = new ElementStructuralStiffnessProvider();
		}

		public PerformanceLogger Logger { get; } = new PerformanceLogger();

		public List<IResultsWriter> Results { get; } = new List<IResultsWriter>();

		public string Status { get; private set; } = "Preparing";

		public void Analyze()
		{
			var watch = new Stopwatch();
			Status = "Running analysis";
			IGlobalVector totalDisplacementsFreeDofs = null;
			for (int iteration = 0; iteration < maxIterations; ++iteration)
			{
				Debug.WriteLine($"Crack propagation step {iteration}");
				//Console.WriteLine($"Crack propagation step {iteration}");
				MpiUtilities.DeclarePerProcess($"Crack propagation step {iteration}");
				//Logger.IncrementAnalysisIteration();

				if (iteration == 0)
				{
					model.Initialize();
					
					watch.Start();
					algebraicModel.OrderDofs();
					watch.Stop();
					solver.Logger.LogTaskDuration("Dof ordering", watch.ElapsedMilliseconds);
					//Logger.LogDofOrderingDuration(watch.ElapsedMilliseconds);

					watch.Restart();
					BuildMatrices();
					watch.Stop();
					solver.Logger.LogTaskDuration("Matrix assembly", watch.ElapsedMilliseconds);
					//Logger.LogMatrixAssemblyDuration(watch.ElapsedMilliseconds);
				}
				else
				{
					model.Update(algebraicModel, totalDisplacementsFreeDofs);
					foreach (ICrack crack in model.GeometryModel.EnumerateDiscontinuities())
					{
						if (termination.MustTerminate(crack))
						{
							Status =
								$"Terminated at iteration {iteration}, because of crack {crack.ID}: {termination.Description}";
							return;
						}
					}

					if (reanalysis)
					{
						watch.Restart();
						algebraicModel.ReorderDofs();
						watch.Stop();
						solver.Logger.LogTaskDuration("Dof ordering", watch.ElapsedMilliseconds);
						//Logger.LogDofOrderingDuration(watch.ElapsedMilliseconds);

						watch.Restart();
						RebuildMatrices();
						watch.Stop();
						solver.Logger.LogTaskDuration("Matrix assembly", watch.ElapsedMilliseconds);
						//Logger.LogMatrixAssemblyDuration(watch.ElapsedMilliseconds);
					}
					else
					{
						watch.Restart();
						algebraicModel.OrderDofs();
						watch.Stop();
						solver.Logger.LogTaskDuration("Dof ordering", watch.ElapsedMilliseconds);
						//Logger.LogDofOrderingDuration(watch.ElapsedMilliseconds);

						watch.Restart();
						BuildMatrices();
						watch.Stop();
						solver.Logger.LogTaskDuration("Matrix assembly", watch.ElapsedMilliseconds);
						//Logger.LogMatrixAssemblyDuration(watch.ElapsedMilliseconds);
					}
				}

				AddLoadsToRhs();

				// Plot domain decomposition data, if necessary
				//if (DDLogger != null) DDLogger.PlotSubdomains(model);

				// Solve the linear system
				//watch.Restart();
				//Console.WriteLine("Solution phase start");
				MpiUtilities.DeclarePerProcess("Solution phase start");
				solver.Solve();
				//Console.WriteLine("Solution phase end");
				MpiUtilities.DeclarePerProcess("Solution phase end");
				#region debug
				//Debug.WriteLine($"norm(u)={solver.LinearSystem.Solution.Norm2()}");
				#endregion
				//watch.Stop();
				//solver.Logger.LogTaskDuration("Solution", watch.ElapsedMilliseconds);
				//Logger.LogSolutionDuration(watch.ElapsedMilliseconds);
				totalDisplacementsFreeDofs = solver.LinearSystem.Solution;
				foreach (IResultsWriter writer in Results)
				{
					writer.WriteResults(algebraicModel, totalDisplacementsFreeDofs);
				}
			}
			Status = $"Termination after all {maxIterations} required iterations completed.";
		}

		private void AddLoadsToRhs()
		{
			// Nodal loads
			algebraicModel.AddToGlobalVector(model.EnumerateNodalLoads, solver.LinearSystem.RhsVector);

			// Dirichlet loads
			IGlobalVector dirichletLoads = algebraicModel.CreateZeroVector();
			algebraicModel.AddToGlobalVector(model.EnumerateDirichletBoundaryConditions, dirichletLoads);
			solver.LinearSystem.RhsVector.SubtractIntoThis(dirichletLoads);
		}

		private void BuildMatrices()
		{
			IGlobalMatrix Kff = algebraicModel.BuildGlobalMatrix(model.EnumerateElements, elementMatrixProvider);
			solver.LinearSystem.Matrix = Kff;
		}

		private void RebuildMatrices()
		{
			IGlobalMatrix previousKff = solver.LinearSystem.Matrix;
			solver.LinearSystem.Matrix = algebraicModel.RebuildGlobalMatrixPartially(
				previousKff, model.EnumerateElements, elementMatrixProvider);
		}
	}
}
