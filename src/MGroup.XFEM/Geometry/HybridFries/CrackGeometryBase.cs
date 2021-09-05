using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Writers;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public abstract class CrackGeometryBase : IHybridFriesCrackDescription
	{
		protected Dictionary<int, double[]> nodalTripleLevelSets;
		private Dictionary<int, double[]> nodalDoubleLevelSets;

		protected CrackGeometryBase(int id)
		{
			ID = id;
		}

		public int ID { get; }

		/// <summary>
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 5.3
		/// </summary>
		/// <param name="allNodes"></param>
		/// <param name="maxDistance"></param>
		/// <returns></returns>
		public HashSet<XNode> FindNodesNearFront(IEnumerable<XNode> allNodes, double maxDistance)
		{
			var nodesNearFront = new HashSet<XNode>();
			foreach (XNode node in allNodes)
			{
				double[] phi = nodalTripleLevelSets[node.ID];
				double r = phi[1];
				if (r <= maxDistance)
				{
					nodesNearFront.Add(node);
				}
			}
			return nodesNearFront;
		}

		public double[] GetNodalLevelSets(XNode node) => nodalDoubleLevelSets[node.ID];

		public double[] GetTripleLevelSetsOf(XNode node) => nodalTripleLevelSets[node.ID];

		/// <summary>
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 5.3 
		/// </summary>
		/// <param name="element"></param>
		public IElementDiscontinuityInteraction Intersect(IXFiniteElement element)
		{
			// Find min, max nodal values of a, b, r
			double mina = double.MaxValue;
			double maxa = double.MinValue;
			double minb = double.MaxValue;
			double maxb = double.MinValue;
			//double minr = double.MaxValue;
			foreach (XNode node in element.Nodes)
			{
				//double[] phi = nodalTripleLevelSets[node.ID];
				//double a = AuxiliaryCoordinateSystems.CalcAlpha(phi); //TODO: Each node's alpha will be recomputed multiple times. I should precompute and cache them.
				//double b = phi[2];

				double[] levelSets = nodalDoubleLevelSets[node.ID];
				double b = levelSets[0];
				double a = levelSets[1];

				if (a < mina)
				{
					mina = a;
				}
				if (a > maxa)
				{
					maxa = a;
				}

				if (b < minb)
				{
					minb = b;
				}
				if (b > maxb)
				{
					maxb = b;
				}

				// This is done by the class that applies enrichments
				//double r = phi[1];
				//if (r < minr)
				//{
				//	minr = r;
				//}
			}

			// Decide on the type of element
			if (minb * maxb < 0) // The element is intersected by the crack or its extension
			{
				if (mina * maxa < 0)
				{
					return new ImplicitCrackElementInteraction(
						this.ID, element, RelativePositionCurveElement.Intersecting, true, null);
				}
				else if (maxa < 0)
				{
					return new ImplicitCrackElementInteraction(
						 this.ID, element, RelativePositionCurveElement.Intersecting, false, null);
				}
			}
			return new NullElementDiscontinuityInteraction(this.ID, element);
		}

		protected void CalcDoubleLevelSets()
		{
			this.nodalDoubleLevelSets = new Dictionary<int, double[]>();
			foreach (var idLevelSetsPair in nodalTripleLevelSets)
			{
				int nodeID = idLevelSetsPair.Key;
				double[] levelSets = idLevelSetsPair.Value;

				double phi = levelSets[2];
				double psi = AuxiliaryCoordinateSystems.CalcAlpha(levelSets);

				nodalDoubleLevelSets[nodeID] = new double[] { phi, psi };
			}
		}
	}
}
