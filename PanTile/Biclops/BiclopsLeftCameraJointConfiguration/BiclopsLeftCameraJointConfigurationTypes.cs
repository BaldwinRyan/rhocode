using System;
using System.Collections.Generic;
using System.ComponentModel;
using Microsoft.Ccr.Core;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using W3C.Soap;

namespace UMass.LPR.BiclopsLeftCameraJointConfiguration
{
	public sealed class Contract
	{
		[DataMember]
		public const string Identifier = "http://schemas.lpr.cs.umass.edu/2009/03/biclopsleftcamerajointconfiguration.html";
	}
}


