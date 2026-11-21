# Third-Party Notices

## RWHE-Calib

Source: https://github.com/ihtishamaliktk/RWHE-Calib

Ali, I., Suominen, O., Gotchev, A., and Morales, E. R. (2019).
“Methods for Simultaneous Robot-World-Hand–Eye Calibration: A Comparative Study.”
*Sensors*, 19(12), 2837. https://doi.org/10.3390/s19122837

Project file paths in the table are relative to `VDF_VDC_MATLAB/`.

| Upstream file | Project file | Adaptation |
| --- | --- | --- |
| `+method/li.m` | `+method/Li.m` | Descriptive output names; formatting |
| `+method/AXZBreproj.m` | `+method/Tabb_rp1.m` | Retains Tabb rp1 with LM; removes unused branches and arguments |
| `+method/AXZB.m` | `+method/Tabb_Zc2.m` | Retains Tabb Zc2 with LM; removes the unused cost-1 solve and alternative branch |
| `auxilaries/createHT.m` | `auxiliaries/createHT.m` | Formatting; original copyright and license retained |
| `auxilaries/invertHT.m` | `auxiliaries/invertRigidTransform.m` | Descriptive function name and explicit function ending |
| `computeErrors.m` | `evaluateCalibration.m` | Retains AX=ZB evaluation; uses SO(3) trace-angle rotation errors; omits unused AXXB and ground-truth branches |

The Li implementation retains its original attribution to Aiguo Li, Lin Wang,
Defeng Wu, and Mili Shah (July 2014).

`createHT.m` retains Peter I. Corke's Robotics Toolbox copyright and LGPL
v3-or-later notice. The LGPL v3 and GPL v3 texts are included in `licenses/`.
Existing third-party notices and terms apply to the respective files.
