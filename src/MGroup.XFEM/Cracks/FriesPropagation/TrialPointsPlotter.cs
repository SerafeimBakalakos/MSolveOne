using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	public class TrialPointsPlotter
	{
		private readonly string directory;
		private int iteration = 0;


		public TrialPointsPlotter(string directory)
		{
			this.directory = directory;
		}

		public void PlotData(IXModel model, LocalizedStressField stressField, List<TrialPointStresses> allTrialPoints)
		{
			PlotNodalStresses(model, stressField);
			PlotTrialPointStressesGlobal(model, allTrialPoints);
			PlotTrialPointStressesPolar(allTrialPoints);
			PlotPropagationVectors(allTrialPoints);

			++iteration;
		}

		private void PlotNodalStresses(IXModel model, LocalizedStressField stressField)
		{
			using (var writer = new VtkPointWriter(Path.Combine(directory, $"stresses_at_nodes_t{iteration}.vtk")))
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
		}

		private void PlotTrialPointStressesGlobal(IXModel model, List<TrialPointStresses> allTrialPoints)
		{
			var stressesAtGlobalPoints = new Dictionary<double[], double[]>();
			foreach (TrialPointStresses trialPoint in allTrialPoints)
			{
				for (int i = 0; i < trialPoint.CoordinatesGlobal.Count; ++i)
				{
					stressesAtGlobalPoints[trialPoint.CoordinatesGlobal[i]] = trialPoint.StressesGlobal[i];
				}
			}

			using (var writer = new VtkPointWriter(Path.Combine(directory, $"stresses_global_at_trial_points_t{iteration}.vtk")))
			{
				if (model.Dimension == 2)
				{
					writer.WriteTensor2DField("stresses", stressesAtGlobalPoints);
				}
				else
				{
					writer.WriteTensor3DField("stresses", stressesAtGlobalPoints);
				}
			}
		}

		private void PlotTrialPointStressesPolar(List<TrialPointStresses> allTrialPoints)
		{
			var pointsGlobal = new List<double[]>();
			var stressesThetaTheta = new List<double>();
			var stressesRTheta = new List<double>();
			foreach (TrialPointStresses trialPoint in allTrialPoints)
			{
				for (int i = 0; i < trialPoint.CoordinatesGlobal.Count; ++i)
				{
					pointsGlobal.Add(trialPoint.CoordinatesGlobal[i]);
					stressesThetaTheta.Add(trialPoint.StressesThetaTheta[i]);
					stressesRTheta.Add(trialPoint.StressesRTheta[i]);
				}
			}

			using (var writer = new VtkPointWriter(Path.Combine(directory, $"stresses_polar_at_trial_points_t{iteration}.vtk")))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("sThetaTheta", stressesThetaTheta);
				writer.WriteScalarField("sRTheta", stressesRTheta);
			}
		}

		private void PlotPropagationVectors(List<TrialPointStresses> allTrialPoints)
		{
			var crackTips = new List<double[]>();
			var propagationVectors = new List<double[]>();
			foreach (TrialPointStresses trialPoint in allTrialPoints)
			{
				var start = Vector.CreateFromArray(trialPoint.CrackTip);
				var end = Vector.CreateFromArray(trialPoint.CoordinatesGlobal[trialPoint.CriticalAngleIndex]);

				Vector diff = end - start;
				diff.ScaleIntoThis(trialPoint.GrowthLength / diff.Norm2());

				crackTips.Add(start.RawData);
				propagationVectors.Add(diff.RawData);
			}

			using (var writer = new VtkPointWriter(Path.Combine(directory, $"propagation_vectors_t{iteration}.vtk")))
			{
				writer.WritePoints(crackTips, true);
				writer.WriteVectorField("propagation_vectors", propagationVectors);
			}
		}
	}
}
