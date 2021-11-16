using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class CrackVicinityNodeSelector : IEnrichedNodeSelector
	{
		private readonly List<IImplicitCrackGeometry> cracks;
		private readonly double maxDistance;

		public CrackVicinityNodeSelector(List<IImplicitCrackGeometry> cracks, double maxDistance)
		{
			this.cracks = new List<IImplicitCrackGeometry>(cracks);
			this.maxDistance = maxDistance;
		}

		public bool CanNodeBeEnriched(INode node)
		{
			foreach (IImplicitCrackGeometry crack in cracks)
			{
				double[] levelSets = crack.GetNodalLevelSets((XNode)node);
				double distanceFromBody = Math.Abs(levelSets[0]);
				bool notCloseToExtension = levelSets[1] < 0;
				double distanceFromTip = Math.Sqrt(levelSets[0] * levelSets[0] + levelSets[1] * levelSets[1]);
				if ((distanceFromTip <= maxDistance) || (notCloseToExtension && (distanceFromBody <= maxDistance)))
				{
					Debug.WriteLine($"Possible enriched node {node.ID} ({node.X}, {node.Y})");
					return true;
				}
			}
			return false;
		}
	}
}
