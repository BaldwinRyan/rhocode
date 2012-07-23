<?xml version ="1.0" encoding ="utf-8" ?>
<xsl:stylesheet version="1.0"
                xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
                xmlns:svc="http://schemas.tempuri.org/2009/03/biclops.html">
<xsl:output method="html"/>

  <xsl:template match="/svc:BiclopsState">

    <html>
      <head>
        <title>Biclops Service</title>
        <link rel="stylesheet" type="text/css" href="/resources/dss/Microsoft.Dss.Runtime.Home.Styles.Common.css" />
      </head>

      <body style="margin:10px">

        <h1>Biclops Service</h1>

        <table border ="0">
          <tr class="odd">
            <th colspan="2">Service State</th>
          </tr>

          <tr class="even">
            <th>Mode</th>
            <td>
              <xsl:value-of select="svc:Mode"/>
            </td>
          </tr>

          <tr class="odd">
            <th>Pan</th>
            <td>
              <xsl:value-of select="svc:JointPositions/svc:double[1]"/>
            </td>
          </tr>

          <tr class="even">
            <th>Tilt</th>
            <td>
              <xsl:value-of select="svc:JointPositions/svc:double[2]"/>
            </td>
          </tr>

          <tr class="odd">
            <th>Pan Commanded</th>
            <td>
              <xsl:value-of select="svc:JointPositionReferences/svc:double[1]"/>
            </td>
          </tr>

          <tr class="even">
            <th>Tilt Commanded</th>
            <td>
              <xsl:value-of select="svc:JointPositionReferences/svc:double[2]"/>
            </td>
          </tr>
          
        </table>

        <BR>
          <BR></BR>
        </BR>
        Use the following form to change the mode of the Biclops:
        <table border="0">

          <tr class="even">
            <th>Mode:</th>
            <td>
              <form name="ModeControl" method="post">
                <select name="NewMode">
                  <option value="3">POSITION</option>
                  <option value="2">HOMING</option>
                  <option value="4">PARKED</option>
                </select>
                <input type="submit" name="Change" value="Change" />
              </form>
            </td>

          </tr>
          
        </table>

        <BR>
          <BR></BR>
        </BR>

        Use the following form to send a position command to Biclops: 
        <form name="PositionControl" method="post">
        <table border="0">
        <tr class="even">
          <th>Pan:</th>
          
            <td>
              <input type="text" name="pos1" class="TextBox">
              </input>
            </td>
          </tr>

        <tr>
          <th>Tilt:</th>
          <td>
            <input type="text" name="pos2" class="TextBox">
            </input>
          </td>
        </tr>

        <tr>
          <td>

          </td>
          <td>
            <input type="submit" name="Command" value="Command" />
          </td>
        </tr>
            
         

        </table>
        </form>
      </body>

    </html>


  </xsl:template>
</xsl:stylesheet>