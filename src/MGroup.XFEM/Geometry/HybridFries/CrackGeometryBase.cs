using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public abstract class CrackGeometryBase : IHybridFriesCrackDescription
	{
		protected Dictionary<int, double[]> nodalLevelSets;

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
				double[] phi = nodalLevelSets[node.ID];
				double r = phi[1];
				if (r <= maxDistance)
				{
					nodesNearFront.Add(node);
				}
			}
			return nodesNearFront;
		}


		public double[] GetLevelSetsOf(XNode node) => nodalLevelSets[node.ID];

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
				double[] phi = nodalLevelSets[node.ID];
				double a = AuxiliaryCoordinateSystems.CalcAlpha(phi); //TODO: Each node's alpha will be recomputed multiple times. I should precompute and cache them.
				if (a < mina)
				{
					mina = a;
				}
				if (a > maxa)
				{
					maxa = a;
				}

				double b = phi[2];
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
			var position = RelativePositionCurveElement.Disjoint;
			bool frontInteractsWithElement = false;
			if (minb * maxb < 0) // The element is intersected by the crack or its extension
			{
				if (mina * maxa < 0)
				{
					position = RelativePositionCurveElement.Intersecting;
					frontInteractsWithElement = true;
				}
				else if (maxa < 0)
				{
					position = RelativePositionCurveElement.Intersecting;
				}
			}
			return new ImplicitCrackElementInteraction(this.ID, element, position, frontInteractsWithElement, null);
		}
	}
}
