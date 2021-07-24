using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.Environments.Mpi;

namespace MGroup.Solvers.DDM.Tests
{
	public enum EnvironmentChoice
	{
		SequentialSharedEnvironment, TplSharedEnvironment, MklEnvironment
	}

	public static class EnvironmentChoiceExtensions
	{
		public static IComputeEnvironment CreateEnvironment(this EnvironmentChoice environmentChoice)
		{
			if (environmentChoice == EnvironmentChoice.SequentialSharedEnvironment) return new SequentialSharedEnvironment();
			else if (environmentChoice == EnvironmentChoice.TplSharedEnvironment) return new TplSharedEnvironment();
			else if (environmentChoice == EnvironmentChoice.MklEnvironment) return new MpiEnvironment();
			else throw new NotImplementedException();
		}
	}
}
