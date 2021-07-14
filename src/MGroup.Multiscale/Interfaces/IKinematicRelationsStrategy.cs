namespace MGroup.Multiscale.Interfaces
{
	public interface IKinematicRelationsStrategy
	{
		double[,] GetNodalKinematicRelationsMatrix(INode boundaryNode);
	}
}
