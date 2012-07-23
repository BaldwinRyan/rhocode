<?xml version="1.0" encoding="utf-8" ?>
<xsl:stylesheet
    version="1.0"
    xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
    xmlns:soap="http://www.w3.org/2003/05/soap-envelope"
    xmlns:dssp="http://schemas.microsoft.com/xw/2004/10/dssp.html"
    xmlns:svc="http://schemas.tempuri.org/2008/07/jointconfiguration.html"
    >

  <xsl:import href="/resources/dss/Microsoft.Dss.Runtime.Home.MasterPage.xslt" />

  <xsl:template match="/">
    <xsl:comment>
      <!-- Service Header Info -->
    </xsl:comment>
    <xsl:variable name="title">
      Biclops Joint Configuration Service
    </xsl:variable>
    <xsl:variable name="serviceName">
      BiclopsJointConfigurationService
    </xsl:variable>
    <xsl:variable name="description">
      Provides information about a Biclops.
    </xsl:variable>

    <xsl:call-template name="MasterPage">
      <xsl:with-param name="serviceName" select="$serviceName" />
      <xsl:with-param name="description" select="$description" />
      <!-- If title is not provided, serviceName will be used instead. -->
      <xsl:with-param name="title">
        <xsl:value-of select="$serviceName" />
        <xsl:if test="$title != ''">
          <xsl:text> - </xsl:text>
          <xsl:value-of select="$title" />
        </xsl:if>
      </xsl:with-param>
      <!-- Possible values for navigation are: 'Open', 'Closed', and 'None'
           'Open' is the default value. -->
      <xsl:with-param name="navigation" select="'Open'" />
      <!-- The contents of head param will be placed just before the </head> tag in html. -->
      <xsl:with-param name="head">
        <link rel="stylesheet" href="resources/BiclopsJointConfiguration.Y2009.M03/UMass.LPR.BiclopsJointConfiguration.Resources.BiclopsJointConfiguration.css" type="text/css" />
        <style type="text/css">
          /* Service-specific stylesheet goes here */
        </style>
        <script language="javascript" type="text/javascript">
          <![CDATA[<!--

/* Service-specific script goes here */

dssRuntime.init = function()
{
  // Add page initialization code here.
  // This function is attached to the window.onload event.
  // Do not override window.onload.
}

//-->     ]]>
        </script>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:template>

  <xsl:template match="/svc:JointConfigurationState">
    <xsl:comment></xsl:comment>
    <form name="DssForm" method="post" onsubmit="return checkform(this);">
      <div class="Content">
        <table width="100%" border="0" cellpadding="5" cellspacing="5">
          <tr>
            <th>Name:</th>
            <td>
              <xsl:value-of select="svc:RobotName"/>
            </td>
          </tr>
          <tr>
            <th>Joint Lock:</th>
            <td>
              <xsl:choose>
                <xsl:when test="svc:Lock='true'">
                  <input type="checkbox" name="lock" checked="yes" value="yes"  />
                </xsl:when>
                <xsl:otherwise>
                  <input type="checkbox" name="lock" value="no"  />
                </xsl:otherwise>
              </xsl:choose>
            </td>
          </tr>
          <tr>
            <th>NumJoints:</th>
            <td>
              <xsl:value-of select="svc:NumJoints"/>
            </td>
          </tr>
          <tr>
            <th>NumLinks:</th>
            <td>
              <xsl:value-of select="svc:NumLinks"/>
            </td>
          </tr>
          <tr>
            <th>Max Delta:</th>
            <td>
              <input type="text" name="delta" class="TextBox">
                <xsl:attribute name="value">
                  <xsl:value-of select="svc:MaxDelta/svc:dat/svc:double[1]"/>
                </xsl:attribute>
              </input>(rad)
            </td>
          </tr>
          <tr>
            <th></th>
            <th>Current</th>
            <th>Desired</th>
            <th>Min</th>
            <th>Max</th>
          </tr>
          <tr>
            <th>Pan:</th>
            <td>
              <xsl:value-of select="svc:JointAngles/svc:dat/svc:double[1]"/>  (rad)
            </td>
            <td>
              <input type="text" name="despan" class="TextBox">
                <xsl:attribute name="value">
                  <xsl:value-of select="svc:DesiredJointAngles/svc:dat/svc:double[1]"/>
                </xsl:attribute>
              </input>(rad)
            </td>
            <td>
              <xsl:value-of select="svc:MinLimits/svc:dat/svc:double[1]"/>  (rad)
            </td>
            <td>
              <xsl:value-of select="svc:MaxLimits/svc:dat/svc:double[1]"/>  (rad)
            </td>
          </tr>
          <tr>
            <th>Tilt:</th>
            <td>
              <xsl:value-of select="svc:JointAngles/svc:dat/svc:double[2]"/>  (rad)
            </td>
            <td>
              <input type="text" name="destilt" class="TextBox">
                <xsl:attribute name="value">
                  <xsl:value-of select="svc:DesiredJointAngles/svc:dat/svc:double[2]"/>
                </xsl:attribute>
              </input>(rad)
            </td>
            <td>
              <xsl:value-of select="svc:MinLimits/svc:dat/svc:double[2]"/> (rad)
            </td>
            <td>
              <xsl:value-of select="svc:MaxLimits/svc:dat/svc:double[2]"/>  (rad)
            </td>
          </tr>
          <tr>
            <td>
              <input type="submit" name="Set" value="Set" />
            </td>
          </tr>
          <tr>
            <th>  </th>
            <th>  </th>
            <th>  </th>
            <th>  </th>
            <th>  </th>
          </tr>
          <tr>
            <td>
              <input type="submit" name="Park" value="Park" />
            </td>
            <td></td>
            <td>
              <input type="submit" name="Home" value="Home" />
            </td>
          </tr>
        </table>
        <table width="100%" border="1" cellpadding="1" cellspacing="1">
          <tr>
            <th>Link #</th>
            <th>DH.Alpha</th>
            <th>DH.A</th>
            <th>DH.D</th>
            <th>DH.Theta</th>
            <th>Link Type</th>
          </tr>
          <xsl:for-each select="svc:DHParams/svc:ArrayOfDouble">
            <tr>
              <th>
                L<xsl:value-of select="position()"/>:
              </th>
              <xsl:for-each select="./svc:double">
                <td>
                  <xsl:value-of select="."/>
                </td>
              </xsl:for-each>
              <td>
                <xsl:value-of select="svc:LinkTypes/svc:string[position()]"/>
              </td>
            </tr>
          </xsl:for-each>
        </table>
      </div>
    </form>
  </xsl:template>
  
  

</xsl:stylesheet>
