using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	public class DefaultGrowthAngleCriterion : IGrowthAngleCriterion
	{
		private readonly double relativeStressRThetaTolerance;

		public DefaultGrowthAngleCriterion(double relativeStressRThetaTolerance)
		{
			this.relativeStressRThetaTolerance = relativeStressRThetaTolerance;
		}

		public void FindPropagationAngle(TrialPointStresses trialPointStresses)
		{
			double tolerance = CalcZeroTolerance(trialPointStresses);
			int resultIndex = -1;
			double maxStressThetaTheta = double.MinValue;
			List<double> stressesThetaTheta = trialPointStresses.StressesThetaTheta;
			List<double> stressesRTheta = trialPointStresses.StressesRTheta;
			for (int i = 0; i < stressesRTheta.Count; ++i)
			{
				if (Math.Abs(stressesRTheta[i]) <= tolerance)
				{
					if (stressesThetaTheta[i] > maxStressThetaTheta)
					{
						resultIndex = i;
						maxStressThetaTheta = stressesThetaTheta[i];
					}
				}
			}

			if (resultIndex == -1)
			{
				throw new Exception("Cound not find any point with: r-theta stress = 0");
			}
			else
			{
				if (stressesThetaTheta[resultIndex] < 0)
				{
					throw new Exception("Cound not find any point with: (r-theta stress = 0) AND (theta-theta stress > 0)");
				}
				trialPointStresses.CriticalAngleIndex = resultIndex;
			}
		}

		private double CalcZeroTolerance(TrialPointStresses trialPointStresses)
		{
			double maxStressRTheta = double.MinValue;
			foreach (double s in trialPointStresses.StressesRTheta)
			{
				if (s > maxStressRTheta)
				{
					maxStressRTheta = s;
				}
			}
			return relativeStressRThetaTolerance * maxStressRTheta;
		}
	}
}
