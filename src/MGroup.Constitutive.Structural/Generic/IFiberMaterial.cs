namespace MGroup.MSolve.Constitutive
{
	/// <summary>
	/// Interface for materials laws implementations to be used in beam sections analysis 
	/// </summary>
	public interface IFiberMaterial : IFiniteElementMaterial
	{
		double Stress { get; }
		double Strain { get; }
		IFiberMaterial Clone(IFiberFiniteElementMaterial parent);
	}
}
