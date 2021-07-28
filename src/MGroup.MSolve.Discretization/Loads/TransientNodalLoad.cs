using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.MSolve.Discretization.Loads
{
	public class TransientNodalLoad : INodalLoad
	{
		private readonly ITransientModel model;
		private readonly double initialAmount;
		private readonly Func<int, double> timeFunc;

		public TransientNodalLoad(INode node, IDofType dof, double initialAmount, 
			Func<int, double> timeFunc, ITransientModel model)
		{
			this.Node = node;
			this.DOF = dof;
			this.initialAmount = initialAmount;
			this.timeFunc = timeFunc;
			this.model = model;
		}

		public IDofType DOF { get; }

		public INode Node { get; }

		public double Amount => timeFunc(model.TimeStep) * initialAmount;
	}
}
