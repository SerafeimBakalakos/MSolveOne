using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Geometry.Tolerances;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public class ImplicitElementIntersectionStrategy2D
	{
		private readonly CrackCurve2D crack;

		public ImplicitElementIntersectionStrategy2D(CrackCurve2D crack)
		{
			this.crack = crack;
		}

		public IMeshTolerance MeshTolerance { get; set; } = new ArbitrarySideMeshTolerance();


		public IElementDiscontinuityInteraction FindIntersectionWithCutElement(IXFiniteElement element, bool isTipElement)
		{
			double tol = MeshTolerance.CalcTolerance(element);
			var allIntersectionsPoints = new HashSet<double[]>();
			IReadOnlyList<ElementEdge> edges = element.Edges;
			Dictionary<int, XNode> nodes = element.NodesAsDictionary();
			for (int i = 0; i < edges.Count; ++i)
			{
				XNode node0 = nodes[edges[i].NodeIDs[0]];
				XNode node1 = nodes[edges[i].NodeIDs[1]];
				double[] node0Natural = edges[i].NodesNatural[0];
				double[] node1Natural = edges[i].NodesNatural[1];
				double levelSet0 = CalcPhiLevelSetNearZero(node0, tol);
				double levelSet1 = CalcPhiLevelSetNearZero(node1, tol);

				if (levelSet0 * levelSet1 > 0.0) continue; // Edge is not intersected
				else if (levelSet0 * levelSet1 < 0.0) // Edge is intersected but not at its nodes
				{
					// The intersection point between these nodes can be found using the linear interpolation, see 
					// Sukumar 2001
					double k = -levelSet0 / (levelSet1 - levelSet0);
					double xi = node0Natural[0] + k * (node1Natural[0] - node0Natural[0]);
					double eta = node0Natural[1] + k * (node1Natural[1] - node0Natural[1]);

					allIntersectionsPoints.Add(new double[] { xi, eta });
				}
				else if ((levelSet0 == 0) && (levelSet1 == 0)) // Curve is tangent to the element. Edge lies on the curve.
				{
					//TODO: also check (DEBUG only) that all other edges are not intersected unless its is at these 2 nodes
					return new OpenLsmElementIntersection2D(crack.ID, element, RelativePositionCurveElement.Conforming,
						isTipElement, new double[][] { node0Natural, node1Natural });
				}
				else if ((levelSet0 == 0) && (levelSet1 != 0)) // Curve runs through a node. Not sure if it is tangent yet.
				{
					allIntersectionsPoints.Add(node0Natural);
				}
				else /*if ((levelSet0 != 0) && (levelSet1 == 0))*/ // Curve runs through a node. Not sure if it is tangent yet.
				{
					allIntersectionsPoints.Add(node1Natural);
				}
			}

			if (allIntersectionsPoints.Count == 1) // Curve is tangent to the element at a single node
			{
				throw new NotImplementedException();
				//TODO: Make sure the intersection point is a node (debug only)
				return new NullElementDiscontinuityInteraction(crack.ID, element);
			}
			else if (allIntersectionsPoints.Count == 2)
			{
				List<double[]> intersections = allIntersectionsPoints.ToList();
				if (isTipElement)
				{
					List<double[]> tipElementintersections = FindIntersectionsWithTipLevelSet(intersections, element);
					RearrangeIntersectionsForConsistentNormal(tipElementintersections);
					return new OpenLsmElementIntersection2D(crack.ID, element, RelativePositionCurveElement.Intersecting,
						isTipElement, tipElementintersections);
				}
				else
				{
					RearrangeIntersectionsForConsistentNormal(intersections);
					return new OpenLsmElementIntersection2D(crack.ID, element, RelativePositionCurveElement.Intersecting,
						isTipElement, intersections);
				}
			}
			else throw new Exception("This should not have happened");
		}

		private double CalcPhiLevelSetNearZero(XNode node, double zeroTolerance)
		{
			double levelSet = crack.GetNodalLevelSets(node)[0];
			if (Math.Abs(levelSet) <= zeroTolerance) return 0.0;
			else return levelSet;
		}

		private double CalcPsiLevelSet(double[] naturalCoords, IXFiniteElement element)
		{
			var point = new XPoint(2);
			point.Element = element;
			point.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(naturalCoords);
			double[] levelSets = crack.InterpolateLevelSets(point);
			return levelSets[1];
		}

		private List<double[]> FindIntersectionsWithTipLevelSet(List<double[]> intersectionsWithCrackBody, IXFiniteElement element)
		{
			double[] p0 = intersectionsWithCrackBody[0];
			double[] p1 = intersectionsWithCrackBody[1];
			double psi0 = CalcPsiLevelSet(p0, element);
			double psi1 = CalcPsiLevelSet(p1, element);
			Debug.Assert((psi0 > 0 && psi1 < 0) || (psi0 < 0 && psi1 > 0));

			double k = -psi0 / (psi1 - psi0);
			double[] tip =
			{
				p0[0] + k * (p1[0] - p0[0]),
				p0[1] + k * (p1[1] - p0[1])
			};

			var results = new List<double[]>(2);
			if (psi0 < 0)
			{
				results.Add(p0);
				results.Add(tip);
			}
			else
			{
				results.Add(p1);
				results.Add(tip);
			}
			return results;
		}

		private void RearrangeIntersectionsForConsistentNormal(List<double[]> intersections)
		{

		}
	}
}
