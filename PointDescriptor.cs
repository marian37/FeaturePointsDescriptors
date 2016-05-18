using System;
using System.Collections.Generic;

namespace FeaturePointsDescriptors
{
	public class PointDescriptor
	{
		private ColorPoint3D point;

		private List<int> intDescriptor;

		private double[] doubleDescriptor;

		private List<ColorPoint3D> smallNeighbourhood;

		private List<ColorPoint3D> bigNeighbourhood;

		private double feature;

		public PointDescriptor (ColorPoint3D point, List<int> descriptor, int numberOfShellBins, List<ColorPoint3D> smallNeighbourhood, List<ColorPoint3D> bigNeighbourhood)
		{
			this.point = point;
			this.intDescriptor = descriptor;
			this.smallNeighbourhood = smallNeighbourhood;
			this.bigNeighbourhood = bigNeighbourhood;

			int count = 0;
			for (int i = 0; i < numberOfShellBins; i++) {
				count += descriptor [i];
			}

			this.doubleDescriptor = new double[descriptor.Count];
			for (int i = 0; i < doubleDescriptor.Length; i++) {
				doubleDescriptor [i] = (double)(descriptor [i]) / count;
			}
		}

		public ColorPoint3D Point {
			get {
				return this.point;
			}
		}

		/*public List<int> IntDescriptor {
			get {
				return this.intDescriptor;
			}
		}*/

		public double[] DoubleDescriptor {
			get {
				return this.doubleDescriptor;
			}
		}

		public List<ColorPoint3D> SmallNeighbourhood {
			get {
				return this.smallNeighbourhood;
			}
		}

		public List<ColorPoint3D> BigNeighbourhood {
			get {
				return this.bigNeighbourhood;
			}
		}			

		public double Feature {
			get {
				return this.feature;
			}
			set {
				feature = value;
			}
		}
	}
}

