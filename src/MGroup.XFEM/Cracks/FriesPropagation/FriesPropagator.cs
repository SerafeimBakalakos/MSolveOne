using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	public class FriesPropagator : IPropagator
	{
		private readonly ICartesianMesh mesh;
		private readonly XModel<IXCrackElement> model;

		public FriesPropagator(XModel<IXCrackElement> model, ICartesianMesh mesh)
		{
			this.model = model;
			this.mesh = mesh;
		}

		public (double growthAngle, double growthLength) Propagate(IAlgebraicModel algebraicModel, 
			IGlobalVector totalDisplacements, double[] crackTip, double[] extensionVector, IEnumerable<IXCrackElement> tipElements)
		{
			List<double[]> trialPointsGlobal = FindTrialPointsGlobal();
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

		private static List<double[]> FindTrialPointsGlobal()
		{
			var points = new List<double[]>();
			points.Add(new double[] { 1.187525, 2.0624750 });
			points.Add(new double[] { 1.229175, 2.0208250 });
			points.Add(new double[] { 1.2708325, 1.9791675 });
			points.Add(new double[] { 1.291665, 1.9375025 });
			points.Add(new double[] { 1.291665, 1.8958350 });
			points.Add(new double[] { 1.291665, 1.8541650 });
			points.Add(new double[] { 1.291665, 1.8124975 });
			points.Add(new double[] { 1.291665, 1.7708325 });
			return points;
		}

		

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
