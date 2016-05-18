using System;
using System.Collections.Generic;

namespace FeaturePointsDescriptors
{
	public class DescriptorMatch
	{
		private PointDescriptor point1;
		private PointDescriptor point2;
		private double match;


		public DescriptorMatch (PointDescriptor point1, PointDescriptor point2, double match)
		{
			this.point1 = point1;
			this.point2 = point2;
			this.match = match;
		}

		public double Match {
			get {
				return this.match;
			}
		}

		public PointDescriptor Point1 {
			get {
				return this.point1;
			}
		}

		public PointDescriptor Point2 {
			get {
				return this.point2;
			}
		}
	}
}

