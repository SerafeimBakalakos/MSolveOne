using System.Collections.Generic;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace ISAAR.MSolve.Logging
{
	public class IncrementalDisplacementsLog
	{
		private readonly IVectorValueExtractor resultsExtractor;
		private readonly List<Table<INode, IDofType, double>> dofDisplacementsPerIter;

		/// <summary>
		/// Initializes a new instance of <see cref="IncrementalDisplacementsLog"/>.
		/// </summary>
		/// <param name="watchDofs">Which freedom degrees to track for each subdomain.</param>
		public IncrementalDisplacementsLog(IList<(INode, IDofType)> watchDofs, IVectorValueExtractor resultsExtractor)
		{
			this.WatchDofs = watchDofs;
			this.resultsExtractor = resultsExtractor;
			this.dofDisplacementsPerIter = new List<Table<INode, IDofType, double>>();
		}

		public IList<(INode, IDofType)> WatchDofs { get; }

		public void StoreDisplacements(IGlobalVector totalDisplacements)
		{
			var currentIterDisplacements = new Table<INode, IDofType, double>();
			foreach ((INode node, IDofType dof) in WatchDofs)
			{
				currentIterDisplacements[node, dof] = resultsExtractor.ExtractSingleValue(totalDisplacements, node, dof);
			}
			dofDisplacementsPerIter.Add(currentIterDisplacements);
		}

		public double GetTotalDisplacement(int iteration, INode node, IDofType dof)
			=> dofDisplacementsPerIter[iteration][node, dof];
	}
}
