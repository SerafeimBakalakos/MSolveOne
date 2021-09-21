using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	public class FriesPropagator : IPropagator
	{
		private const double minGrowthAngle = -75.0 * Math.PI / 180.0;
		private const double maxGrowthAngle = +75.0 * Math.PI / 180.0;

		private readonly ICartesianMesh mesh;
		private readonly double growthLength;
		private readonly XModel<IXCrackElement> model;
		private readonly int numTrialPointsPerTip;
		private readonly double trialPointRadius;

		public FriesPropagator(XModel<IXCrackElement> model, ICartesianMesh mesh, 
			double growthLength, double trialPointRadius, int numTrialPointsPerTip)
		{
			this.model = model;
			this.mesh = mesh;

			this.growthLength = growthLength;
			this.trialPointRadius = trialPointRadius;

			if (numTrialPointsPerTip < 3)
			{
				throw new ArgumentException();
			}
			this.numTrialPointsPerTip = numTrialPointsPerTip;
		}

		public (double growthAngle, double growthLength) Propagate(IAlgebraicModel algebraicModel, 
			IGlobalVector totalDisplacements, ICrackTipSystem crackTipSystem)
		{
			List<double[]> trialPointsGlobal = FindTrialPointsGlobal(crackTipSystem);
			var stressField = new LocalizedStressField(model.Dimension, algebraicModel, totalDisplacements);
			var stressesGlobalAtPoints = new Dictionary<double[], double[]>();
			foreach (double[] pointGlobal in trialPointsGlobal)
			{
				(int elementID, double[] pointNatural) = mesh.FindElementContaining(pointGlobal);
				IXCrackElement element = model.Elements[elementID];
				stressesGlobalAtPoints[pointGlobal] = stressField.CalcStressesAtPoint(pointNatural, element);
			}

			PlotNodalStresses(stressField, stressesGlobalAtPoints);
			return (0, 0);
		}

		private List<double[]> FindTrialPointsGlobal(ICrackTipSystem crackTipSystem)
		{
			//TODO: generalize this for 3D
			var points = new List<double[]>();
			double dTheta = (maxGrowthAngle - minGrowthAngle) / (numTrialPointsPerTip - 1);
			for (int i = 0; i < numTrialPointsPerTip; ++i)
			{
				double theta = minGrowthAngle + i * dTheta;
				double[] point = crackTipSystem.ExtendTowards(theta, trialPointRadius);
				points.Add(point);
			}
			return points;

			//var points = new List<double[]>();
			//points.Add(new double[] { 1.187525, 2.0624750 });
			//points.Add(new double[] { 1.229175, 2.0208250 });
			//points.Add(new double[] { 1.2708325, 1.9791675 });
			//points.Add(new double[] { 1.291665, 1.9375025 });
			//points.Add(new double[] { 1.291665, 1.8958350 });
			//points.Add(new double[] { 1.291665, 1.8541650 });
			//points.Add(new double[] { 1.291665, 1.8124975 });
			//points.Add(new double[] { 1.291665, 1.7708325 });
			//return points;
		}

		////TODO: These should be done by the crack front coordinate system. Same for allocating points. Then
		//private (Matrix Q, Vector b) CalcLocalCartesianSystem(double[] crackTip, double[] extensionVector)
		//{
		//	HERE
		//}

		private void PlotNodalStresses(LocalizedStressField stressField, Dictionary<double[], double[]> stressesAtTrialPoints)
		{
			using (var writer = new VtkPointWriter(@"C:\Users\Serafeim\Desktop\xfem 3d\plots\edge_crack_2D_hybrid\stresses_at_nodes.vtk"))
			{
				var nodalStresses = new Dictionary<double[], double[]>();
				foreach (var pair in stressField.StoredNodalStresses)
				{
					int nodeID = pair.Key;
					double[] stresses = pair.Value;
					double[] coords = model.Nodes[nodeID].Coordinates;
					nodalStresses[coords] = stresses;
				}

				if (model.Dimension == 2)
				{
					writer.WriteTensor2DField("stresses", nodalStresses);
				}
				else
				{
					writer.WriteTensor3DField("stresses", nodalStresses);
				}
			}

			using (var writer = new VtkPointWriter(@"C:\Users\Serafeim\Desktop\xfem 3d\plots\edge_crack_2D_hybrid\stresses_at_trial_points.vtk"))
			{
				if (model.Dimension == 2)
				{
					writer.WriteTensor2DField("stresses", stressesAtTrialPoints);
				}
				else
				{
					writer.WriteTensor3DField("stresses", stressesAtTrialPoints);
				}
			}
		}
	}
}
