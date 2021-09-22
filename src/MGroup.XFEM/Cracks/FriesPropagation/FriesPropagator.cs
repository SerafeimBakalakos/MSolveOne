using System;
using System.Collections.Generic;
using System.Diagnostics;
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
		/// <summary>
		/// 75 or 70.529 degrees.
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 6.1
		/// </summary>
		private const double growthAngleLimitDefault = 70.529; 

		private readonly int dimension;
		private double growthAngleMax;
		private double growthAngleMin;
		private readonly double growthLength;
		private readonly ICartesianMesh mesh;
		private readonly IGrowthAngleCriterion growthAngleCriterion;
		private readonly XModel<IXCrackElement> model;
		private readonly int numTrialPointsPerTip;
		private readonly double trialPointRadius;
		private int iteration;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="model"></param>
		/// <param name="mesh"></param>
		/// <param name="growthLength"></param>
		/// <param name="trialPointRadius"></param>
		/// <param name="numTrialPointsPerTip"></param>
		/// <param name="growthAngleLimit">
		/// In degrees, not radd. Trials points will be generated between 
		/// (-<paramref name="growthAngleLimit"/>, + <paramref name="growthAngleLimit"/>)
		/// </param>
		public FriesPropagator(XModel<IXCrackElement> model, ICartesianMesh mesh, IGrowthAngleCriterion growthAngleCriterion, 
			double growthLength, double trialPointRadius, int numTrialPointsPerTip, 
			double growthAngleLimit = growthAngleLimitDefault)
		{
			this.model = model;
			this.mesh = mesh;
			this.growthAngleCriterion = growthAngleCriterion;
			if ((model.Dimension != 2) && (model.Dimension != 3))
			{
				throw new ArgumentException("The dimension of the model must be 2 or 3");
			}
			this.dimension = model.Dimension;

			this.growthLength = growthLength;
			this.trialPointRadius = trialPointRadius;

			if (numTrialPointsPerTip < 3)
			{
				throw new ArgumentException();
			}
			this.numTrialPointsPerTip = numTrialPointsPerTip;

			this.growthAngleMin = -growthAngleLimit * Math.PI / 180.0;
			this.growthAngleMax = +growthAngleLimit * Math.PI / 180.0;
		}

		public (double[] growthAngles, double[] growthLengths) Propagate(
			IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements, ICrackTipSystem[] crackTipSystems)
		{
			int numTips = crackTipSystems.Length;
			var growthAngles = new double[numTips];
			var hoopStresses = new double[numTips];
			double maxHoopStress = double.MinValue;
			for (int i = 0; i < numTips; ++i)
			{
				(double growthAngle, double stressThetaTheta) = 
					PropagateTip(algebraicModel, totalDisplacements, crackTipSystems[i]);
				Debug.Assert(stressThetaTheta > 0);

				growthAngles[i] = growthAngle;
				hoopStresses[i] = stressThetaTheta;
				maxHoopStress = Math.Max(maxHoopStress, stressThetaTheta);
			}

			var growthLengths = new double[numTips];
			for (int i = 0; i < numTips; ++i)
			{
				growthLengths[i] = this.growthLength * hoopStresses[i] / maxHoopStress;
			}

			return (growthAngles, growthLengths);
		}

		private (double growthAngle, double stressThetaTheta) PropagateTip(IAlgebraicModel algebraicModel, 
			IGlobalVector totalDisplacements, ICrackTipSystem crackTipSystem)
		{
			(List<double> anglesTheta, List<double[]> coordsGlobal) = FindTrialPointsGlobal(crackTipSystem);
			var stressField = new LocalizedStressField(dimension, algebraicModel, totalDisplacements); //TODO: use the same object for all tips.
			var stressesThetaTheta = new List<double>();
			var stressesRTheta = new List<double>();

			#region plot
			var stressesGlobalAtPoints = new Dictionary<double[], double[]>();
			#endregion

			for (int i = 0; i < anglesTheta.Count; ++i)
			{
				(int elementID, double[] coordsNatural) = mesh.FindElementContaining(coordsGlobal[i]);
				IXCrackElement element = model.Elements[elementID];
				double[] globalStressTensor = stressField.CalcStressesAtPoint(coordsNatural, element);
				(double stressThetaTheta, double stressRTheta) = 
					CalcPolarStresses(globalStressTensor, anglesTheta[i], crackTipSystem);
				stressesThetaTheta.Add(stressThetaTheta);
				stressesRTheta.Add(stressRTheta);

				#region plot
				stressesGlobalAtPoints[coordsGlobal[i]] = globalStressTensor;
				#endregion
			}

			PlotStresses(stressField, coordsGlobal, stressesGlobalAtPoints, stressesThetaTheta, stressesRTheta);

			int pointIndex = growthAngleCriterion.FindIndexOfPropagationAngle(anglesTheta, stressesThetaTheta, stressesRTheta);
			return (anglesTheta[pointIndex], stressesThetaTheta[pointIndex]);
		}

		/// <summary>
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// eq. 6.1, 6.2
		/// </summary>
		/// <param name="globalStressTensor"></param>
		/// <param name="crackTipSystem"></param>
		/// <returns></returns>
		private (double stressThetaTheta, double stressRTheta) CalcPolarStresses(
			double[] globalStressTensor, double theta, ICrackTipSystem crackTipSystem)
		{
			// Stress tensor in local cartesian system (its x1 axis coincides with the extension vector)
			double[] localCartesianStressTensor = crackTipSystem.RotateGlobalStressTensor(globalStressTensor);
			double snn, stt, snt;
			if (dimension == 2)
			{
				// Stress tensor is [stt snn snt]
				stt = localCartesianStressTensor[0];
				snn = localCartesianStressTensor[1];
				snt = localCartesianStressTensor[2];
			}
			else
			{
				// Stress tensor is [stt sqq snn stq sqn snt]
				stt = localCartesianStressTensor[0];
				snn = localCartesianStressTensor[2];
				snt = localCartesianStressTensor[5];
			}

			// Stress in polar system
			double cos = Math.Cos(theta);
			double sin = Math.Sin(theta);
			double cos2 = Math.Cos(2 * theta);
			double sin2 = Math.Sin(2 * theta);
			double sThetaTheta = snn * sin * sin + stt * cos * cos - snt * sin2;
			double sRTheta = sin * cos * (snn - stt) + snt * cos2;

			return (sThetaTheta, sRTheta);
		}

		private (List<double> anglesTheta, List<double[]> coordsGlobal) FindTrialPointsGlobal(ICrackTipSystem crackTipSystem)
		{
			//TODO: generalize this for 3D
			var anglesTheta = new List<double>();
			var coordsGlobal = new List<double[]>();
			double dTheta = (growthAngleMax - growthAngleMin) / (numTrialPointsPerTip - 1);
			for (int i = 0; i < numTrialPointsPerTip; ++i)
			{
				double theta = growthAngleMin + i * dTheta;
				double[] point = crackTipSystem.ExtendTowards(theta, trialPointRadius);

				anglesTheta.Add(theta);
				coordsGlobal.Add(point);
			}
			return (anglesTheta, coordsGlobal);

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

		private void PlotStresses(LocalizedStressField stressField, List<double[]> pointsGlobal,
			Dictionary<double[], double[]> stressesAtTrialPoints, List<double> stressesThetaTheta, List<double> stressesRTheta)
		{
			string directory = @"C:\Users\Serafeim\Desktop\xfem 3d\plots\edge_crack_2D_hybrid\";
			using (var writer = new VtkPointWriter(directory + $"stresses_at_nodes_t{iteration}.vtk"))
			{
				var nodalStresses = new Dictionary<double[], double[]>();
				foreach (var pair in stressField.StoredNodalStresses)
				{
					int nodeID = pair.Key;
					double[] stresses = pair.Value;
					double[] coords = model.Nodes[nodeID].Coordinates;
					nodalStresses[coords] = stresses;
				}

				if (dimension == 2)
				{
					writer.WriteTensor2DField("stresses", nodalStresses);
				}
				else
				{
					writer.WriteTensor3DField("stresses", nodalStresses);
				}
			}

			using (var writer = new VtkPointWriter(directory + $"stresses_at_trial_points_t{iteration}.vtk"))
			{
				if (dimension == 2)
				{
					writer.WriteTensor2DField("stresses", stressesAtTrialPoints);
				}
				else
				{
					writer.WriteTensor3DField("stresses", stressesAtTrialPoints);
				}
			}

			using (var writer = new VtkPointWriter(directory + $"stresses_polar_at_trial_points_t{iteration}.vtk"))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("sThetaTheta", stressesThetaTheta);
				writer.WriteScalarField("sRTheta", stressesRTheta);
			}

			++iteration;
		}
	}
}
