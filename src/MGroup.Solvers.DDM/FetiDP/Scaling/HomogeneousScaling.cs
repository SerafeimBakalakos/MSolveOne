using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.LagrangeMultipliers;

namespace MGroup.Solvers.DDM.FetiDP.Scaling
{
	public class HomogeneousScaling : IFetiDPScaling
	{
		private readonly IComputeEnvironment environment;
		private readonly IModel model;
		private readonly Func<int, FetiDPSubdomainDofs> getSubdomainDofs;
		private readonly FetiDPReanalysisOptions reanalysis;
		private readonly ConcurrentDictionary<int, double[]> inverseMultiplicities = new ConcurrentDictionary<int, double[]>();

		public HomogeneousScaling(IComputeEnvironment environment, IModel model, 
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, ICrossPointStrategy crossPointStrategy, 
			FetiDPReanalysisOptions reanalysis)
		{
			this.environment = environment;
			this.model = model;
			this.getSubdomainDofs = getSubdomainDofs;
			this.reanalysis = reanalysis;

			if (!(crossPointStrategy is FullyRedundantLagranges))
			{
				throw new NotImplementedException();
			}
		}

		public IDictionary<int, DiagonalMatrix> SubdomainMatricesWbr { get; } = new ConcurrentDictionary<int, DiagonalMatrix>();

		public void CalcScalingMatrices()
		{
			bool isFirstAnalysis = inverseMultiplicities.Count == 0;
			Action<int> calcSubdomainScaling = subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.RhsVectors
					|| reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing inverse multiplicities of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing inverse multiplicities of subdomain {subdomainID}");
					#endregion

					FetiDPSubdomainDofs dofs = getSubdomainDofs(subdomainID);
					var Wbr = new double[dofs.DofsBoundaryRemainderToRemainder.Length];
					foreach ((int nodeID, int dofID, int brIndex) in dofs.DofOrderingBoundaryRemainder)
					{
						int multiplicity = model.GetNode(nodeID).Subdomains.Count;
						Wbr[brIndex] = 1.0 / multiplicity;
					}
					inverseMultiplicities[subdomainID] = Wbr;
					SubdomainMatricesWbr[subdomainID] = DiagonalMatrix.CreateFromArray(Wbr);
				}
			};
			environment.DoPerNode(calcSubdomainScaling);
		}

		public void ScaleBoundaryRhsVector(int subdomainID, Vector boundaryRemainderRhsVector)
		{
			double[] coefficients = inverseMultiplicities[subdomainID];
			Debug.Assert(boundaryRemainderRhsVector.Length == coefficients.Length);
			for (int i = 0; i < coefficients.Length; i++)
			{
				boundaryRemainderRhsVector[i] *= coefficients[i];
			}
		}
	}
}
