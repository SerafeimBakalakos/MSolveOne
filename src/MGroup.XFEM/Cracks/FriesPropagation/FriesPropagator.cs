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

		#region debug
		public TrialPointsPlotter plotter;
		#endregion

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
			var stressField = new LocalizedStressField(dimension, algebraicModel, totalDisplacements);
			var allTrialPoints = new List<TrialPointStresses>();
			for (int i = 0; i < numTips; ++i)
			{
				TrialPointStresses trialPointStresses = PropagateTip(crackTipSystems[i], stressField);
				allTrialPoints.Add(trialPointStresses);

				int criticalPointIdx = trialPointStresses.CriticalAngleIndex;
				growthAngles[i] = trialPointStresses.Angles[criticalPointIdx];
				double hoopStress = trialPointStresses.StressesThetaTheta[criticalPointIdx];
				Debug.Assert(hoopStress > 0);

				hoopStresses[i] = hoopStress;
				maxHoopStress = Math.Max(maxHoopStress, hoopStress);
			}

			var growthLengths = new double[numTips];
			for (int i = 0; i < numTips; ++i)
			{
				growthLengths[i] = this.growthLength * hoopStresses[i] / maxHoopStress;
				allTrialPoints[i].GrowthLength = growthLengths[i];
			}

			if (plotter != null)
			{
				plotter.PlotData(model, stressField, allTrialPoints);
			}
			return (growthAngles, growthLengths);
		}

		private TrialPointStresses PropagateTip(ICrackTipSystem crackTipSystem, LocalizedStressField stressField)
		{
			(List<double> anglesTheta, List<double[]> coordsGlobal) = FindTrialPointsGlobal(crackTipSystem);
			var result = new TrialPointStresses(crackTipSystem.TipCoordsGlobal);
			for (int i = 0; i < anglesTheta.Count; ++i)
			{
				(int elementID, double[] coordsNatural) = mesh.FindElementContaining(coordsGlobal[i]);
				IXCrackElement element = model.Elements[elementID];
				double[] globalStressTensor = stressField.CalcStressesAtPoint(coordsNatural, element);
				(double stressThetaTheta, double stressRTheta) =
					CalcPolarStresses(globalStressTensor, anglesTheta[i], crackTipSystem);

				result.Angles.Add(anglesTheta[i]);
				result.CoordinatesGlobal.Add(coordsGlobal[i]);
				result.StressesGlobal.Add(globalStressTensor);
				result.StressesThetaTheta.Add(stressThetaTheta);
				result.StressesRTheta.Add(stressRTheta);
			}

			growthAngleCriterion.FindPropagationAngle(result);
			return result;
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
		}
	}
}
