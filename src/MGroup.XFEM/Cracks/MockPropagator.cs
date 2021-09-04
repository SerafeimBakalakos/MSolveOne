using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Cracks
{
	/// <summary>
	/// Used in tests. Only for propagation from one tip for now.
	/// </summary>
	public class MockPropagator: IPropagatorOLD
	{
		/// <summary>
		/// In the local polar coordinate system defined at the crack tip.
		/// </summary>
		private int iteration;

		public MockPropagator(double[] angles, double[] lengths)
		{
			Logger = new PropagationLogger();
			for (int t = 0; t < angles.Length; ++t)
			{
				Logger.GrowthAngles.Add(angles[t]);
				Logger.GrowthLengths.Add(lengths[t]);
			}
		}

		public MockPropagator(string anglesLengthsPath)
		{
			Logger = ReadFromFile(anglesLengthsPath);
			iteration = 0;
		}

		/// <summary>
		/// 
		/// </summary>
		/// <param name="logger">
		/// Intermediate crack propagation data that were gathered by a previous analysis and will be enforced now.</param>
		public MockPropagator(PropagationLogger logger)
		{
			this.Logger = logger;
			for (int i = 0; i < logger.GrowthAngles.Count; ++i)
			{
				logger.SIFsMode1.Add(0.0);
				logger.SIFsMode2.Add(0.0);
			}

			iteration = 0;
		}

		public PropagationLogger Logger { get; }

		
		public (double growthAngle, double growthLength) Propagate(
			IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements,
			double[] crackTipGlobal, TipCoordinateSystemExplicit tipSystem, IEnumerable<IXCrackElement> tipElements)
		{
			if (iteration >= Logger.GrowthLengths.Count)
			{
				throw new IndexOutOfRangeException($"Only {Logger.GrowthLengths.Count} iterations have been recorder.");
			}
			double angle = Logger.GrowthAngles[iteration];
			double length = Logger.GrowthLengths[iteration];

			++iteration;
			return (angle, length);
		}

		/// <summary>
		/// The file must have the number of iteration in the first line. Then in each line it must have the growth angle in the
		/// local cartesian system around the crack tip, the growth length SIF Mode I and SIF mode II of a new iteration, 
		/// separated by a space. No empty lines at the end.
		/// </summary>
		/// <param name="anglesLengthsPath"></param>
		private PropagationLogger ReadFromFile(string anglesLengthsPath)
		{
			using (var reader = new StreamReader(anglesLengthsPath))
			{
				var logger = new PropagationLogger();
				int numIterations = int.Parse(reader.ReadLine());
				for (int i = 0; i < numIterations; ++i)
				{
					string[] words = reader.ReadLine().Split(' ');
					if (words.Length != 4) throw new IOException("Each line must have the growth angle, growth length," 
						+" SIF Mode I and SIF Mode II of an iteration, separated by a space");
					logger.GrowthAngles.Add(double.Parse(words[0]));
					logger.GrowthLengths.Add(double.Parse(words[1]));
					logger.SIFsMode1.Add(double.Parse(words[2]));
					logger.SIFsMode2.Add(double.Parse(words[3]));
				}
				return logger;
			}
		}
	}
}
