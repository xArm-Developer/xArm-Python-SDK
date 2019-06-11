<!-- vscode-markdown-toc -->
- [1. <a name='class__XArmAPI__'></a>class __XArmAPI__](#1-a-nameclassxarmapiaclass-xarmapi)
  - [1.1. <a name='Attributes__'></a>__Attributes__](#11-a-nameattributesaattributes)
    - [1.1.1. <a name='angles__'></a>__angles__](#111-a-nameanglesaangles)
    - [1.1.2. <a name='axis__'></a>__axis__](#112-a-nameaxisaaxis)
    - [1.1.3. <a name='cmd_num__'></a>__cmd_num__](#113-a-namecmdnumacmdnum)
    - [1.1.4. <a name='collision_sensitivity__'></a>__collision_sensitivity__](#114-a-namecollisionsensitivityacollisionsensitivity)
    - [1.1.5. <a name='connected__'></a>__connected__](#115-a-nameconnectedaconnected)
    - [1.1.6. <a name='core__'></a>__core__](#116-a-namecoreacore)
    - [1.1.7. <a name='default_is_radian__'></a>__default_is_radian__](#117-a-namedefaultisradianadefaultisradian)
    - [1.1.8. <a name='device_type__'></a>__device_type__](#118-a-namedevicetypeadevicetype)
    - [1.1.9. <a name='error_code__'></a>__error_code__](#119-a-nameerrorcodeaerrorcode)
    - [1.1.10. <a name='gravity_direction__'></a>__gravity_direction__](#1110-a-namegravitydirectionagravitydirection)
    - [1.1.11. <a name='has_err_warn__'></a>__has_err_warn__](#1111-a-namehaserrwarnahaserrwarn)
    - [1.1.12. <a name='has_error__'></a>__has_error__](#1112-a-namehaserrorahaserror)
    - [1.1.13. <a name='has_warn__'></a>__has_warn__](#1113-a-namehaswarnahaswarn)
    - [1.1.14. <a name='joint_acc_limit__'></a>__joint_acc_limit__](#1114-a-namejointacclimitajointacclimit)
    - [1.1.15. <a name='joint_speed_limit__'></a>__joint_speed_limit__](#1115-a-namejointspeedlimitajointspeedlimit)
    - [1.1.16. <a name='joints_torque__'></a>__joints_torque__](#1116-a-namejointstorqueajointstorque)
    - [1.1.17. <a name='last_used_angles__'></a>__last_used_angles__](#1117-a-namelastusedanglesalastusedangles)
    - [1.1.18. <a name='last_used_joint_acc__'></a>__last_used_joint_acc__](#1118-a-namelastusedjointaccalastusedjointacc)
    - [1.1.19. <a name='last_used_joint_speed__'></a>__last_used_joint_speed__](#1119-a-namelastusedjointspeedalastusedjointspeed)
    - [1.1.20. <a name='last_used_position__'></a>__last_used_position__](#1120-a-namelastusedpositionalastusedposition)
    - [1.1.21. <a name='last_used_tcp_acc__'></a>__last_used_tcp_acc__](#1121-a-namelastusedtcpaccalastusedtcpacc)
    - [1.1.22. <a name='last_used_tcp_speed__'></a>__last_used_tcp_speed__](#1122-a-namelastusedtcpspeedalastusedtcpspeed)
    - [1.1.23. <a name='master_id__'></a>__master_id__](#1123-a-namemasteridamasterid)
    - [1.1.24. <a name='mode__'></a>__mode__](#1124-a-namemodeamode)
    - [1.1.25. <a name='motor_brake_states__'></a>__motor_brake_states__](#1125-a-namemotorbrakestatesamotorbrakestates)
    - [1.1.26. <a name='motor_enable_states__'></a>__motor_enable_states__](#1126-a-namemotorenablestatesamotorenablestates)
    - [1.1.27. <a name='position__'></a>__position__](#1127-a-namepositionaposition)
    - [1.1.28. <a name='slave_id__'></a>__slave_id__](#1128-a-nameslaveidaslaveid)
    - [1.1.29. <a name='sn__'></a>__sn__](#1129-a-namesnasn)
    - [1.1.30. <a name='state__'></a>__state__](#1130-a-namestateastate)
    - [1.1.31. <a name='tcp_acc_limit__'></a>__tcp_acc_limit__](#1131-a-nametcpacclimitatcpacclimit)
    - [1.1.32. <a name='tcp_load__'></a>__tcp_load__](#1132-a-nametcploadatcpload)
    - [1.1.33. <a name='tcp_offset__'></a>__tcp_offset__](#1133-a-nametcpoffsetatcpoffset)
    - [1.1.34. <a name='tcp_speed_limit__'></a>__tcp_speed_limit__](#1134-a-nametcpspeedlimitatcpspeedlimit)
    - [1.1.35. <a name='teach_sensitivity__'></a>__teach_sensitivity__](#1135-a-nameteachsensitivityateachsensitivity)
    - [1.1.36. <a name='version__'></a>__version__](#1136-a-nameversionaversion)
    - [1.1.37. <a name='version_number__'></a>__version_number__](#1137-a-nameversionnumberaversionnumber)
    - [1.1.38. <a name='warn_code__'></a>__warn_code__](#1138-a-namewarncodeawarncode)
  - [1.2. <a name='Methods__'></a>__Methods__](#12-a-namemethodsamethods)
    - [1.2.1. <a name='def____init____selfportNoneis_radianFalsedo_not_openFalsekwargs:'></a>def __\__init__\__(self, port=None, is_radian=False, do_not_open=False, **kwargs):](#121-a-namedefinitselfportnoneisradianfalsedonotopenfalsekwargsadef-initself-portnone-isradianfalse-donotopenfalse-kwargs)
    - [1.2.2. <a name='def__clean_conf__self:'></a>def __clean_conf__(self):](#122-a-namedefcleanconfselfadef-cleanconfself)
    - [1.2.3. <a name='def__clean_error__self:'></a>def __clean_error__(self):](#123-a-namedefcleanerrorselfadef-cleanerrorself)
    - [1.2.4. <a name='def__clean_gripper_error__self:'></a>def __clean_gripper_error__(self):](#124-a-namedefcleangrippererrorselfadef-cleangrippererrorself)
    - [1.2.5. <a name='def__clean_warn__self:'></a>def __clean_warn__(self):](#125-a-namedefcleanwarnselfadef-cleanwarnself)
    - [1.2.6. <a name='def__connect__selfportNonebaudrateNonetimeoutNone:'></a>def __connect__(self, port=None, baudrate=None, timeout=None):](#126-a-namedefconnectselfportnonebaudratenonetimeoutnoneadef-connectself-portnone-baudratenone-timeoutnone)
    - [1.2.7. <a name='def__disconnect__self:'></a>def __disconnect__(self):](#127-a-namedefdisconnectselfadef-disconnectself)
    - [1.2.8. <a name='def__emergency_stop__self:'></a>def __emergency_stop__(self):](#128-a-namedefemergencystopselfadef-emergencystopself)
    - [1.2.9. <a name='def__get_cgpio_analog__selfionumNone:'></a>def __get_cgpio_analog__(self, ionum=None):](#129-a-namedefgetcgpioanalogselfionumnoneadef-getcgpioanalogself-ionumnone)
    - [1.2.10. <a name='def__get_cgpio_digital__selfionumNone:'></a>def __get_cgpio_digital__(self, ionum=None):](#1210-a-namedefgetcgpiodigitalselfionumnoneadef-getcgpiodigitalself-ionumnone)
    - [1.2.11. <a name='def__get_cgpio_state__self:'></a>def __get_cgpio_state__(self):](#1211-a-namedefgetcgpiostateselfadef-getcgpiostateself)
    - [1.2.12. <a name='def__get_cmdnum__self:'></a>def __get_cmdnum__(self):](#1212-a-namedefgetcmdnumselfadef-getcmdnumself)
    - [1.2.13. <a name='def__get_err_warn_code__selfshowFalse:'></a>def __get_err_warn_code__(self, show=False):](#1213-a-namedefgeterrwarncodeselfshowfalseadef-geterrwarncodeself-showfalse)
    - [1.2.14. <a name='def__get_forward_kinematics__selfanglesinput_is_radianNonereturn_is_radianNone:'></a>def __get_forward_kinematics__(self, angles, input_is_radian=None, return_is_radian=None):](#1214-a-namedefgetforwardkinematicsselfanglesinputisradiannonereturnisradiannoneadef-getforwardkinematicsself-angles-inputisradiannone-returnisradiannone)
    - [1.2.15. <a name='def__get_gripper_err_code__self:'></a>def __get_gripper_err_code__(self):](#1215-a-namedefgetgrippererrcodeselfadef-getgrippererrcodeself)
    - [1.2.16. <a name='def__get_gripper_position__self:'></a>def __get_gripper_position__(self):](#1216-a-namedefgetgripperpositionselfadef-getgripperpositionself)
    - [1.2.17. <a name='def__get_inverse_kinematics__selfposeinput_is_radianNonereturn_is_radianNone:'></a>def __get_inverse_kinematics__(self, pose, input_is_radian=None, return_is_radian=None):](#1217-a-namedefgetinversekinematicsselfposeinputisradiannonereturnisradiannoneadef-getinversekinematicsself-pose-inputisradiannone-returnisradiannone)
    - [1.2.18. <a name='def__get_is_moving__self:'></a>def __get_is_moving__(self):](#1218-a-namedefgetismovingselfadef-getismovingself)
    - [1.2.19. <a name='def__get_position__selfis_radianNone:'></a>def __get_position__(self, is_radian=None):](#1219-a-namedefgetpositionselfisradiannoneadef-getpositionself-isradiannone)
    - [1.2.20. <a name='def__get_servo_angle__selfservo_idNoneis_radianNone:'></a>def __get_servo_angle__(self, servo_id=None, is_radian=None):](#1220-a-namedefgetservoangleselfservoidnoneisradiannoneadef-getservoangleself-servoidnone-isradiannone)
    - [1.2.21. <a name='def__get_servo_debug_msg__selfshowFalse:'></a>def __get_servo_debug_msg__(self, show=False):](#1221-a-namedefgetservodebugmsgselfshowfalseadef-getservodebugmsgself-showfalse)
    - [1.2.22. <a name='def__get_state__self:'></a>def __get_state__(self):](#1222-a-namedefgetstateselfadef-getstateself)
    - [1.2.23. <a name='def__get_tgpio_analog__selfionumNone:'></a>def __get_tgpio_analog__(self, ionum=None):](#1223-a-namedefgettgpioanalogselfionumnoneadef-gettgpioanalogself-ionumnone)
    - [1.2.24. <a name='def__get_tgpio_digital__selfionumNone:'></a>def __get_tgpio_digital__(self, ionum=None):](#1224-a-namedefgettgpiodigitalselfionumnoneadef-gettgpiodigitalself-ionumnone)
    - [1.2.25. <a name='def__get_version__self:'></a>def __get_version__(self):](#1225-a-namedefgetversionselfadef-getversionself)
    - [1.2.26. <a name='def__is_joint_limit__selfjointis_radianNone:'></a>def __is_joint_limit__(self, joint, is_radian=None):](#1226-a-namedefisjointlimitselfjointisradiannoneadef-isjointlimitself-joint-isradiannone)
    - [1.2.27. <a name='def__is_tcp_limit__selfposeis_radianNone:'></a>def __is_tcp_limit__(self, pose, is_radian=None):](#1227-a-namedefistcplimitselfposeisradiannoneadef-istcplimitself-pose-isradiannone)
    - [1.2.28. <a name='def__motion_enable__selfenableTrueservo_idNone:'></a>def __motion_enable__(self, enable=True, servo_id=None):](#1228-a-namedefmotionenableselfenabletrueservoidnoneadef-motionenableself-enabletrue-servoidnone)
    - [1.2.29. <a name='def__move_arc_lines__selfpathsis_radianNonetimes1first_pause_time0.1repeat_pause_time0automatic_calibrationTruespeedNonemvaccNonemvtimeNonewaitFalse:'></a>def __move_arc_lines__(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0, automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):](#1229-a-namedefmovearclinesselfpathsisradiannonetimes1firstpausetime01repeatpausetime0automaticcalibrationtruespeednonemvaccnonemvtimenonewaitfalseadef-movearclinesself-paths-isradiannone-times1-firstpausetime01-repeatpausetime0-automaticcalibrationtrue-speednone-mvaccnone-mvtimenone-waitfalse)
    - [1.2.30. <a name='def__move_circle__selfpose1pose2percentspeedNonemvaccNonemvtimeNoneis_radianNonewaitFalsetimeoutNonekwargs:'></a>def __move_circle__(self, pose1, pose2, percent, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):](#1230-a-namedefmovecircleselfpose1pose2percentspeednonemvaccnonemvtimenoneisradiannonewaitfalsetimeoutnonekwargsadef-movecircleself-pose1-pose2-percent-speednone-mvaccnone-mvtimenone-isradiannone-waitfalse-timeoutnone-kwargs)
    - [1.2.31. <a name='def__move_gohome__selfspeedNonemvaccNonemvtimeNoneis_radianNonewaitFalsetimeoutNone:'></a>def __move_gohome__(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):](#1231-a-namedefmovegohomeselfspeednonemvaccnonemvtimenoneisradiannonewaitfalsetimeoutnoneadef-movegohomeself-speednone-mvaccnone-mvtimenone-isradiannone-waitfalse-timeoutnone)
    - [1.2.32. <a name='def__register_cmdnum_changed_callback__selfcallbackNone:'></a>def __register_cmdnum_changed_callback__(self, callback=None):](#1232-a-namedefregistercmdnumchangedcallbackselfcallbacknoneadef-registercmdnumchangedcallbackself-callbacknone)
    - [1.2.33. <a name='def__register_connect_changed_callback__selfcallbackNone:'></a>def __register_connect_changed_callback__(self, callback=None):](#1233-a-namedefregisterconnectchangedcallbackselfcallbacknoneadef-registerconnectchangedcallbackself-callbacknone)
    - [1.2.34. <a name='def__register_error_warn_changed_callback__selfcallbackNone:'></a>def __register_error_warn_changed_callback__(self, callback=None):](#1234-a-namedefregistererrorwarnchangedcallbackselfcallbacknoneadef-registererrorwarnchangedcallbackself-callbacknone)
    - [1.2.35. <a name='def__register_mode_changed_callback__selfcallbackNone:'></a>def __register_mode_changed_callback__(self, callback=None):](#1235-a-namedefregistermodechangedcallbackselfcallbacknoneadef-registermodechangedcallbackself-callbacknone)
    - [1.2.36. <a name='def__register_mtable_mtbrake_changed_callback__selfcallbackNone:'></a>def __register_mtable_mtbrake_changed_callback__(self, callback=None):](#1236-a-namedefregistermtablemtbrakechangedcallbackselfcallbacknoneadef-registermtablemtbrakechangedcallbackself-callbacknone)
    - [1.2.37. <a name='def__register_report_callback__selfcallbackNonereport_cartesianTruereport_jointsTruereport_stateTruereport_error_codeTruereport_warn_codeTruereport_mtableTruereport_mtbrakeTruereport_cmd_numTrue:'></a>def __register_report_callback__(self, callback=None, report_cartesian=True, report_joints=True, report_state=True, report_error_code=True, report_warn_code=True, report_mtable=True, report_mtbrake=True, report_cmd_num=True):](#1237-a-namedefregisterreportcallbackselfcallbacknonereportcartesiantruereportjointstruereportstatetruereporterrorcodetruereportwarncodetruereportmtabletruereportmtbraketruereportcmdnumtrueadef-registerreportcallbackself-callbacknone-reportcartesiantrue-reportjointstrue-reportstatetrue-reporterrorcodetrue-reportwarncodetrue-reportmtabletrue-reportmtbraketrue-reportcmdnumtrue)
    - [1.2.38. <a name='def__register_report_location_callback__selfcallbackNonereport_cartesianTruereport_jointsTrue:'></a>def __register_report_location_callback__(self, callback=None, report_cartesian=True, report_joints=True):](#1238-a-namedefregisterreportlocationcallbackselfcallbacknonereportcartesiantruereportjointstrueadef-registerreportlocationcallbackself-callbacknone-reportcartesiantrue-reportjointstrue)
    - [1.2.39. <a name='def__register_state_changed_callback__selfcallbackNone:'></a>def __register_state_changed_callback__(self, callback=None):](#1239-a-namedefregisterstatechangedcallbackselfcallbacknoneadef-registerstatechangedcallbackself-callbacknone)
    - [1.2.40. <a name='def__release_cmdnum_changed_callback__selfcallbackNone:'></a>def __release_cmdnum_changed_callback__(self, callback=None):](#1240-a-namedefreleasecmdnumchangedcallbackselfcallbacknoneadef-releasecmdnumchangedcallbackself-callbacknone)
    - [1.2.41. <a name='def__release_connect_changed_callback__selfcallbackNone:'></a>def __release_connect_changed_callback__(self, callback=None):](#1241-a-namedefreleaseconnectchangedcallbackselfcallbacknoneadef-releaseconnectchangedcallbackself-callbacknone)
    - [1.2.42. <a name='def__release_error_warn_changed_callback__selfcallbackNone:'></a>def __release_error_warn_changed_callback__(self, callback=None):](#1242-a-namedefreleaseerrorwarnchangedcallbackselfcallbacknoneadef-releaseerrorwarnchangedcallbackself-callbacknone)
    - [1.2.43. <a name='def__release_mode_changed_callback__selfcallbackNone:'></a>def __release_mode_changed_callback__(self, callback=None):](#1243-a-namedefreleasemodechangedcallbackselfcallbacknoneadef-releasemodechangedcallbackself-callbacknone)
    - [1.2.44. <a name='def__release_mtable_mtbrake_changed_callback__selfcallbackNone:'></a>def __release_mtable_mtbrake_changed_callback__(self, callback=None):](#1244-a-namedefreleasemtablemtbrakechangedcallbackselfcallbacknoneadef-releasemtablemtbrakechangedcallbackself-callbacknone)
    - [1.2.45. <a name='def__release_report_callback__selfcallbackNone:'></a>def __release_report_callback__(self, callback=None):](#1245-a-namedefreleasereportcallbackselfcallbacknoneadef-releasereportcallbackself-callbacknone)
    - [1.2.46. <a name='def__release_report_location_callback__selfcallbackNone:'></a>def __release_report_location_callback__(self, callback=None):](#1246-a-namedefreleasereportlocationcallbackselfcallbacknoneadef-releasereportlocationcallbackself-callbacknone)
    - [1.2.47. <a name='def__release_state_changed_callback__selfcallbackNone:'></a>def __release_state_changed_callback__(self, callback=None):](#1247-a-namedefreleasestatechangedcallbackselfcallbacknoneadef-releasestatechangedcallbackself-callbacknone)
    - [1.2.48. <a name='def__reset__selfspeedNonemvaccNonemvtimeNoneis_radianNonewaitFalsetimeoutNone:'></a>def __reset__(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):](#1248-a-namedefresetselfspeednonemvaccnonemvtimenoneisradiannonewaitfalsetimeoutnoneadef-resetself-speednone-mvaccnone-mvtimenone-isradiannone-waitfalse-timeoutnone)
    - [1.2.49. <a name='def__run_blockly_app__selfpath:'></a>def __run_blockly_app__(self, path):](#1249-a-namedefrunblocklyappselfpathadef-runblocklyappself-path)
    - [1.2.50. <a name='def__save_conf__self:'></a>def __save_conf__(self):](#1250-a-namedefsaveconfselfadef-saveconfself)
    - [1.2.51. <a name='def__send_cmd_sync__selfcommandNone:'></a>def __send_cmd_sync__(self, command=None):](#1251-a-namedefsendcmdsyncselfcommandnoneadef-sendcmdsyncself-commandnone)
    - [1.2.52. <a name='def__set_cgpio_analog__selfionumvalue:'></a>def __set_cgpio_analog__(self, ionum, value):](#1252-a-namedefsetcgpioanalogselfionumvalueadef-setcgpioanalogself-ionum-value)
    - [1.2.53. <a name='def__set_cgpio_digital__selfionumvalue:'></a>def __set_cgpio_digital__(self, ionum, value):](#1253-a-namedefsetcgpiodigitalselfionumvalueadef-setcgpiodigitalself-ionum-value)
    - [1.2.54. <a name='def__set_cgpio_digital_input_function__selfionumfun:'></a>def __set_cgpio_digital_input_function__(self, ionum, fun):](#1254-a-namedefsetcgpiodigitalinputfunctionselfionumfunadef-setcgpiodigitalinputfunctionself-ionum-fun)
    - [1.2.55. <a name='def__set_cgpio_digital_output_function__selfionumfun:'></a>def __set_cgpio_digital_output_function__(self, ionum, fun):](#1255-a-namedefsetcgpiodigitaloutputfunctionselfionumfunadef-setcgpiodigitaloutputfunctionself-ionum-fun)
    - [1.2.56. <a name='def__set_collision_sensitivity__selfvalue:'></a>def __set_collision_sensitivity__(self, value):](#1256-a-namedefsetcollisionsensitivityselfvalueadef-setcollisionsensitivityself-value)
    - [1.2.57. <a name='def__set_gravity_direction__selfdirection:'></a>def __set_gravity_direction__(self, direction):](#1257-a-namedefsetgravitydirectionselfdirectionadef-setgravitydirectionself-direction)
    - [1.2.58. <a name='def__set_gripper_enable__selfenable:'></a>def __set_gripper_enable__(self, enable):](#1258-a-namedefsetgripperenableselfenableadef-setgripperenableself-enable)
    - [1.2.59. <a name='def__set_gripper_mode__selfmode:'></a>def __set_gripper_mode__(self, mode):](#1259-a-namedefsetgrippermodeselfmodeadef-setgrippermodeself-mode)
    - [1.2.60. <a name='def__set_gripper_position__selfposwaitFalsespeedNoneauto_enableFalsetimeoutNone:'></a>def __set_gripper_position__(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):](#1260-a-namedefsetgripperpositionselfposwaitfalsespeednoneautoenablefalsetimeoutnoneadef-setgripperpositionself-pos-waitfalse-speednone-autoenablefalse-timeoutnone)
    - [1.2.61. <a name='def__set_gripper_speed__selfspeed:'></a>def __set_gripper_speed__(self, speed):](#1261-a-namedefsetgripperspeedselfspeedadef-setgripperspeedself-speed)
    - [1.2.62. <a name='def__set_joint_jerk__selfjerkis_radianNone:'></a>def __set_joint_jerk__(self, jerk, is_radian=None):](#1262-a-namedefsetjointjerkselfjerkisradiannoneadef-setjointjerkself-jerk-isradiannone)
    - [1.2.63. <a name='def__set_joint_maxacc__selfaccis_radianNone:'></a>def __set_joint_maxacc__(self, acc, is_radian=None):](#1263-a-namedefsetjointmaxaccselfaccisradiannoneadef-setjointmaxaccself-acc-isradiannone)
    - [1.2.64. <a name='def__set_mode__selfmode0:'></a>def __set_mode__(self, mode=0):](#1264-a-namedefsetmodeselfmode0adef-setmodeself-mode0)
    - [1.2.65. <a name='def__set_mount_direction__selfbase_tilt_degrotation_degis_radianNone:'></a>def __set_mount_direction__(self, base_tilt_deg, rotation_deg, is_radian=None):](#1265-a-namedefsetmountdirectionselfbasetiltdegrotationdegisradiannoneadef-setmountdirectionself-basetiltdeg-rotationdeg-isradiannone)
    - [1.2.66. <a name='def__set_pause_time__selfsltimewaitFalse:'></a>def __set_pause_time__(self, sltime, wait=False):](#1266-a-namedefsetpausetimeselfsltimewaitfalseadef-setpausetimeself-sltime-waitfalse)
    - [1.2.67. <a name='def__set_position__selfxNoneyNonezNonerollNonepitchNoneyawNoneradiusNonespeedNonemvaccNonemvtimeNonerelativeFalseis_radianNonewaitFalsetimeoutNonekwargs:'></a>def __set_position__(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, **kwargs):](#1267-a-namedefsetpositionselfxnoneynoneznonerollnonepitchnoneyawnoneradiusnonespeednonemvaccnonemvtimenonerelativefalseisradiannonewaitfalsetimeoutnonekwargsadef-setpositionself-xnone-ynone-znone-rollnone-pitchnone-yawnone-radiusnone-speednone-mvaccnone-mvtimenone-relativefalse-isradiannone-waitfalse-timeoutnone-kwargs)
    - [1.2.68. <a name='def__set_servo_angle__selfservo_idNoneangleNonespeedNonemvaccNonemvtimeNonerelativeFalseis_radianNonewaitFalsetimeoutNonekwargs:'></a>def __set_servo_angle__(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, **kwargs):](#1268-a-namedefsetservoangleselfservoidnoneanglenonespeednonemvaccnonemvtimenonerelativefalseisradiannonewaitfalsetimeoutnonekwargsadef-setservoangleself-servoidnone-anglenone-speednone-mvaccnone-mvtimenone-relativefalse-isradiannone-waitfalse-timeoutnone-kwargs)
    - [1.2.69. <a name='def__set_servo_angle_j__selfanglesspeedNonemvaccNonemvtimeNoneis_radianNonekwargs:'></a>def __set_servo_angle_j__(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):](#1269-a-namedefsetservoanglejselfanglesspeednonemvaccnonemvtimenoneisradiannonekwargsadef-setservoanglejself-angles-speednone-mvaccnone-mvtimenone-isradiannone-kwargs)
    - [1.2.70. <a name='def__set_servo_attach__selfservo_idNone:'></a>def __set_servo_attach__(self, servo_id=None):](#1270-a-namedefsetservoattachselfservoidnoneadef-setservoattachself-servoidnone)
    - [1.2.71. <a name='def__set_servo_detach__selfservo_idNone:'></a>def __set_servo_detach__(self, servo_id=None):](#1271-a-namedefsetservodetachselfservoidnoneadef-setservodetachself-servoidnone)
    - [1.2.72. <a name='def__set_state__selfstate0:'></a>def __set_state__(self, state=0):](#1272-a-namedefsetstateselfstate0adef-setstateself-state0)
    - [1.2.73. <a name='def__set_tcp_jerk__selfjerk:'></a>def __set_tcp_jerk__(self, jerk):](#1273-a-namedefsettcpjerkselfjerkadef-settcpjerkself-jerk)
    - [1.2.74. <a name='def__set_tcp_load__selfweightcenter_of_gravity:'></a>def __set_tcp_load__(self, weight, center_of_gravity):](#1274-a-namedefsettcploadselfweightcenterofgravityadef-settcploadself-weight-centerofgravity)
    - [1.2.75. <a name='def__set_tcp_maxacc__selfacc:'></a>def __set_tcp_maxacc__(self, acc):](#1275-a-namedefsettcpmaxaccselfaccadef-settcpmaxaccself-acc)
    - [1.2.76. <a name='def__set_tcp_offset__selfoffsetis_radianNone:'></a>def __set_tcp_offset__(self, offset, is_radian=None):](#1276-a-namedefsettcpoffsetselfoffsetisradiannoneadef-settcpoffsetself-offset-isradiannone)
    - [1.2.77. <a name='def__set_teach_sensitivity__selfvalue:'></a>def __set_teach_sensitivity__(self, value):](#1277-a-namedefsetteachsensitivityselfvalueadef-setteachsensitivityself-value)
    - [1.2.78. <a name='def__set_tgpio_digital__selfionumvalue:'></a>def __set_tgpio_digital__(self, ionum, value):](#1278-a-namedefsettgpiodigitalselfionumvalueadef-settgpiodigitalself-ionum-value)
    - [1.2.79. <a name='def__shutdown_system__selfvalue1:'></a>def __shutdown_system__(self, value=1):](#1279-a-namedefshutdownsystemselfvalue1adef-shutdownsystemself-value1)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->xArm-Python-SDK API Documentation: class XArmAPI in module xarm.wrapper.xarm_api

##  1. <a name='class__XArmAPI__'></a>class __XArmAPI__
****************************************

###  1.1. <a name='Attributes__'></a>__Attributes__
****************************************
####  1.1.1. <a name='angles__'></a>__angles__
```
Servo angles
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: [angle1(° or rad), angle2(° or rad), ..., anglen(° or rad)]
```

####  1.1.2. <a name='axis__'></a>__axis__
```
Axis number, only available in socket way and enable_report is True and report_type is 'rich'
```

####  1.1.3. <a name='cmd_num__'></a>__cmd_num__
```
Number of command caches in the controller
```

####  1.1.4. <a name='collision_sensitivity__'></a>__collision_sensitivity__
```
The sensitivity value of collision, only available in socket way and  enable_report is True and report_type is 'rich'

:return: 0~5
```

####  1.1.5. <a name='connected__'></a>__connected__
```
Connection status
```

####  1.1.6. <a name='core__'></a>__core__
```
Core layer API, set only for advanced developers, please do not use
Ex:
    self.core.move_line(...)
    self.core.move_lineb(...)
    self.core.move_joint(...)
    ...
```

####  1.1.7. <a name='default_is_radian__'></a>__default_is_radian__
```
The default unit is radians or not
```

####  1.1.8. <a name='device_type__'></a>__device_type__
```
Device type, only available in socket way and  enable_report is True and report_type is 'rich'
```

####  1.1.9. <a name='error_code__'></a>__error_code__
```
Controller error code. See the controller error code documentation for details.
```

####  1.1.10. <a name='gravity_direction__'></a>__gravity_direction__
```
gravity direction, only available in socket way and enable_report is True and report_type is 'rich'
:return:
```

####  1.1.11. <a name='has_err_warn__'></a>__has_err_warn__
```
Contorller have an error or warning or not

:return: True/False
```

####  1.1.12. <a name='has_error__'></a>__has_error__
```
Controller have an error or not
```

####  1.1.13. <a name='has_warn__'></a>__has_warn__
```
Controller have an warnning or not
```

####  1.1.14. <a name='joint_acc_limit__'></a>__joint_acc_limit__
```
Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: [min_joint_acc(°/s^2 or rad/s^2), max_joint_acc(°/s^2 or rad/s^2)]
```

####  1.1.15. <a name='joint_speed_limit__'></a>__joint_speed_limit__
```
Joint speed limit,  only available in socket way and enable_report is True and report_type is 'rich'
Note:
    1. If self.default_is_radian is True, the returned value is in radians
    
:return: [min_joint_speed(°/s or rad/s), max_joint_speed(°/s or rad/s)]
```

####  1.1.16. <a name='joints_torque__'></a>__joints_torque__
```
Joints torque, only available in socket way and  enable_report is True and report_type is 'rich'

:return: [joint-1, ....]
```

####  1.1.17. <a name='last_used_angles__'></a>__last_used_angles__
```
The last used servo angles, default value of parameter angle of interface set_servo_angle
Note:
    1. If self.default_is_radian is True, the returned value is in radians
    2. self.set_servo_angle(servo_id=1, angle=75) <==> self.set_servo_angle(angle=[75] + self.last_used_angles[1:])
    3. self.set_servo_angle(servo_id=5, angle=30) <==> self.set_servo_angle(angle=self.last_used_angles[:4] + [30] + self.last_used_angles[5:])

:return: [angle1(° or rad), angle2(° or rad), ..., angle7(° or rad)]
```

####  1.1.18. <a name='last_used_joint_acc__'></a>__last_used_joint_acc__
```
The last used joint acceleration, default value of parameter mvacc of interface set_servo_angle
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: acceleration (°/s^2 or rad/s^2)
```

####  1.1.19. <a name='last_used_joint_speed__'></a>__last_used_joint_speed__
```
The last used joint speed, default value of parameter speed of interface set_servo_angle
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: speed (°/s or rad/s)
```

####  1.1.20. <a name='last_used_position__'></a>__last_used_position__
```
The last used cartesion position, default value of parameter x/y/z/roll/pitch/yaw of interface set_position
Note:
    1. If self.default_is_radian is True, the returned value (only roll/pitch/yaw) is in radians
    2. self.set_position(x=300) <==> self.set_position(x=300, *last_used_position[1:])
    2. self.set_position(roll=-180) <==> self.set_position(x=self.last_used_position[:3], roll=-180, *self.last_used_position[4:])

:return: [x(mm), y(mm), z(mm), roll(° or rad), pitch(° or rad), yaw(° or rad)]
```

####  1.1.21. <a name='last_used_tcp_acc__'></a>__last_used_tcp_acc__
```
The last used cartesion acceleration, default value of parameter mvacc of interface set_position/move_circle

:return: acceleration (mm/s^2)
```

####  1.1.22. <a name='last_used_tcp_speed__'></a>__last_used_tcp_speed__
```
The last used cartesion speed, default value of parameter speed of interface set_position/move_circle

:return: speed (mm/s)
```

####  1.1.23. <a name='master_id__'></a>__master_id__
```
Master id, only available in socket way and enable_report is True and report_type is 'rich'
```

####  1.1.24. <a name='mode__'></a>__mode__
```
xArm mode，only available in socket way and  enable_report is True

:return: 
    0: position control mode
    1: servo motion mode
    2: joint teaching mode
    3: cartesian teaching mode (invalid)
```

####  1.1.25. <a name='motor_brake_states__'></a>__motor_brake_states__
```
Motor brake state list, only available in socket way and  enable_report is True and report_type is 'rich'
Note:
    For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.

:return: [motor-1-brake-state, motor-2-..., motor-3-..., motor-4-..., motor-5-..., motor-6-..., motor-7-..., reserved]
    motor-{i}-brake-state:
        0: enable
        1: disable
```

####  1.1.26. <a name='motor_enable_states__'></a>__motor_enable_states__
```
Motor enable state list, only available in socket way and  enable_report is True and report_type is 'rich'
Note:
    For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.
    
:return: [motor-1-enable-state, motor-2-..., motor-3-..., motor-4-..., motor-5-..., motor-6-..., motor-7-..., reserved]
    motor-{i}-enable-state:
        0: disable
        1: enable
```

####  1.1.27. <a name='position__'></a>__position__
```
Cartesion position
Note:
    1. If self.default_is_radian is True, the returned value (only roll/pitch/yaw) is in radians

return: [x(mm), y(mm), z(mm), roll(° or rad), pitch(° or rad), yaw(° or rad)]
```

####  1.1.28. <a name='slave_id__'></a>__slave_id__
```
Slave id, only available in socket way and enable_report is True and report_type is 'rich'
```

####  1.1.29. <a name='sn__'></a>__sn__
```
xArm sn
```

####  1.1.30. <a name='state__'></a>__state__
```
xArm state

:return: 
    1: in motion
    2: sleeping
    3: suspended
    4: stopping
```

####  1.1.31. <a name='tcp_acc_limit__'></a>__tcp_acc_limit__
```
Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 

:return: [min_tcp_acc(mm/s^2), max_tcp_acc(mm/s^2)]
```

####  1.1.32. <a name='tcp_load__'></a>__tcp_load__
```
xArm tcp load, only available in socket way and  enable_report is True and report_type is 'rich'

:return: [weight, center of gravity] 
    such as: [weight(kg), [x(mm), y(mm), z(mm)]]
```

####  1.1.33. <a name='tcp_offset__'></a>__tcp_offset__
```
Cartesion position offset, only available in socket way and enable_report is True 
Note:
    1. If self.default_is_radian is True, the returned value(roll_offset/pitch_offset/yaw_offset) is in radians

:return: [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or rad), pitch_offset(° or rad), yaw_offset(° or rad)]
```

####  1.1.34. <a name='tcp_speed_limit__'></a>__tcp_speed_limit__
```
Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 

:return: [min_tcp_acc(mm/s), max_tcp_acc(mm/s)]
```

####  1.1.35. <a name='teach_sensitivity__'></a>__teach_sensitivity__
```
The sensitivity value of drag and teach, only available in socket way and  enable_report is True and report_type is 'rich'

:return: 0~5
```

####  1.1.36. <a name='version__'></a>__version__
```
xArm version
```

####  1.1.37. <a name='version_number__'></a>__version_number__
```
Frimware version number
:return: (major_version_number, minor_version_number, revision_version_number)
```

####  1.1.38. <a name='warn_code__'></a>__warn_code__
```
Controller warn code. See the controller warn code documentation for details.
```

****************************************
###  1.2. <a name='Methods__'></a>__Methods__
****************************************
####  1.2.1. <a name='def____init____selfportNoneis_radianFalsedo_not_openFalsekwargs:'></a>def __\__init__\__(self, port=None, is_radian=False, do_not_open=False, **kwargs):

```
The API wrapper of xArm
Note: Orientation of attitude angle
    roll: rotate around the X axis
    pitch: rotate around the Y axis
    yaw: rotate around the Z axis

:param port: ip-address(such as '192.168.1.185')
    Note: this parameter is required if parameter do_not_open is False
:param is_radian: set the default unit is radians or not, default is False
    Note: (aim of design)
        1. Default value for unified interface parameters
        2: Unification of the external unit system
        3. For compatibility with previous interfaces
    Note: the conversion of degree (°) and radians (rad)
        * 1 rad == 57.29577951308232 °
        * 1 ° == 0.017453292519943295 rad
        * 1 rad/s == 57.29577951308232 °/s
        * 1 °/s == 0.017453292519943295 rad/s
        * 1 rad/s^2 == 57.29577951308232 °/s^2
        * 1 °/s^2 == 0.017453292519943295 rad/s^2
        * 1 rad/s^3 == 57.29577951308232 °/s^3
        * 1 °/s^3 == 0.017453292519943295 rad/s^3
    Note: This parameter determines the value of the property self.default_is_radian 
    Note: This parameter determines the default value of the interface with the is_radian/input_is_radian/return_is_radian parameter
       The list of affected interfaces is as follows:
            1. method: get_position
            2. method: set_position
            3. method: get_servo_angle
            4. method: set_servo_angle
            5. method: set_servo_angle_j
            6. method: move_gohome
            7. method: reset
            8. method: set_tcp_offset
            9. method: set_joint_jerk
            10. method: set_joint_maxacc
            11. method: get_inverse_kinematics
            12. method: get_forward_kinematics
            13. method: is_tcp_limit
            14. method: is_joint_limit
            15. method: get_params
            16: method: move_arc_lines
            17: method: move_circle
    Note: This parameter determines the default return type for some interfaces (such as the position, velocity, and acceleration associated with the return angle arc).
        The affected attributes are as follows:
            1. property: position
            2. property: last_used_position
            3. property: angles
            4. property: last_used_angles
            5. property: last_used_joint_speed
            6. property: last_used_joint_acc
            7. property: tcp_offset
:param do_not_open: do not open, default is False, if true, you need to manually call the connect interface.
:param kwargs: keyword parameters, generally do not need to set
    baudrate: serial baudrate, invalid, reserved.
    timeout: serial timeout, invalid, reserved.
    filters: serial port filters, invalid, reserved.
    check_tcp_limit: check the tcp param value out of limit or not, default is True
        Note: only check the param roll/pitch/yaw of the interface `set_position`/`move_arc_lines`
    check_joint_limit: check the joint param value out of limit or not, default is True
        Note: only check the param angle of the interface `set_servo_angle` and the param angles of the interface `set_servo_angle_j`
    check_cmdnum_limit: check the cmdnum out of limit or not, default is True
        Note: only available in the interface `set_position`/`set_servo_angle`/`move_circle`/`move_arc_lines`
    check_is_ready: check if the arm is in motion, default is True
        Note: only available in the interface `set_position`/`set_servo_angle`/`set_servo_angle_j`/`move_circle`/`move_gohome`/`move_arc_lines`
```

####  1.2.2. <a name='def__clean_conf__self:'></a>def __clean_conf__(self):

```
Clean current config and restore system default settings
Note:
    1. This interface will clear the current settings and restore to the original settings (system default settings)

:return: code
    code: See the API code documentation for details.
```

####  1.2.3. <a name='def__clean_error__self:'></a>def __clean_error__(self):

```
Clean the error, need to be manually enabled motion and set state after clean error

:return: code
    code: See the API code documentation for details.
```

####  1.2.4. <a name='def__clean_gripper_error__self:'></a>def __clean_gripper_error__(self):

```
Clean the gripper error

:return: code
    code: See the Gripper code documentation for details.
```

####  1.2.5. <a name='def__clean_warn__self:'></a>def __clean_warn__(self):

```
Clean the warn

:return: code
    code: See the API code documentation for details.
```

####  1.2.6. <a name='def__connect__selfportNonebaudrateNonetimeoutNone:'></a>def __connect__(self, port=None, baudrate=None, timeout=None):

```
Connect to xArm

:param port: port name or the ip address, default is the value when initializing an instance
:param baudrate: baudrate, only available in serial way, default is the value when initializing an instance
:param timeout: timeout, only available in serial way, default is the value when initializing an instance
```

####  1.2.7. <a name='def__disconnect__self:'></a>def __disconnect__(self):

```
Disconnect
```

####  1.2.8. <a name='def__emergency_stop__self:'></a>def __emergency_stop__(self):

```
Emergency stop (set_state(4) -> motion_enable(True) -> set_state(0))
Note:
    1. This interface does not automatically clear the error. If there is an error, you need to handle it according to the error code.
```

####  1.2.9. <a name='def__get_cgpio_analog__selfionumNone:'></a>def __get_cgpio_analog__(self, ionum=None):

```
Get the analog value of the specified Controller GPIO
:param ionum: 0 or 1 or None(both 0 and 1), default is None
:return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.10. <a name='def__get_cgpio_digital__selfionumNone:'></a>def __get_cgpio_digital__(self, ionum=None):

```
Get the digital value of the specified Controller GPIO

:param ionum: 0~7 or None(both 0~7), default is None
:return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.11. <a name='def__get_cgpio_state__self:'></a>def __get_cgpio_state__(self):

```
Get the state of the Controller GPIO
:return: code, states
    code: See the API code documentation for details.
    states: [...]
        states[0]: contorller gpio module state
            states[0] == 0: normal
            states[0] == 1：wrong
            states[0] == 6：communication failure
        states[1]: controller gpio module error code
            states[1] == 0: normal
            states[1] != 0：error code
        states[2]: digital input functional gpio state
            Note: digital-i-input functional gpio state = states[2] >> i & 0x01
        states[3]: digital input configuring gpio state
            Note: digital-i-input configuring gpio state = states[3] >> i & 0x01
        states[4]: digital output functional gpio state
            Note: digital-i-output functional gpio state = states[4] >> i & 0x01
        states[5]: digital output configuring gpio state
            Note: digital-i-output configuring gpio state = states[5] >> i & 0x01
        states[6]: analog-0 input value
        states[7]: analog-1 input value
        states[8]: analog-0 output value
        states[9]: analog-1 output value
        states[10]: digital input functional info, [digital-0-input-functional-mode, ... digital-7-input-functional-mode]
        states[11]: digital output functional info, [digital-0-output-functional-mode, ... digital-7-output-functional-mode]
```

####  1.2.12. <a name='def__get_cmdnum__self:'></a>def __get_cmdnum__(self):

```
Get the cmd count in cache
:return: tuple((code, cmd_num)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.13. <a name='def__get_err_warn_code__selfshowFalse:'></a>def __get_err_warn_code__(self, show=False):

```
Get the controller error and warn code

:param show: show the detail info if True
:return: tuple((code, [error_code, warn_code])), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    error_code: See the controller error code documentation for details.
    warn_code: See the controller warn code documentation for details.
```

####  1.2.14. <a name='def__get_forward_kinematics__selfanglesinput_is_radianNonereturn_is_radianNone:'></a>def __get_forward_kinematics__(self, angles, input_is_radian=None, return_is_radian=None):

```
Get forward kinematics

:param angles: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm
:param input_is_radian: the param angles value is in radians or not, default is self.default_is_radian
:param return_is_radian: the returned value is in radians or not, default is self.default_is_radian
:return: tuple((code, pose)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)] or []
        Note: the roll/pitch/yaw value is radians if return_is_radian is True, else °
```

####  1.2.15. <a name='def__get_gripper_err_code__self:'></a>def __get_gripper_err_code__(self):

```
Get the gripper error code

:return: tuple((code, err_code)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    err_code: See the Gripper code documentation for details.
```

####  1.2.16. <a name='def__get_gripper_position__self:'></a>def __get_gripper_position__(self):

```
Get the gripper position

:return: tuple((code, pos)), only when code is 0, the returned result is correct.
    code: See the Gripper code documentation for details.
```

####  1.2.17. <a name='def__get_inverse_kinematics__selfposeinput_is_radianNonereturn_is_radianNone:'></a>def __get_inverse_kinematics__(self, pose, input_is_radian=None, return_is_radian=None):

```
Get inverse kinematics

:param pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    Note: the roll/pitch/yaw unit is radian if input_is_radian is True, else °
:param input_is_radian: the param pose value(only roll/pitch/yaw) is in radians or not, default is self.default_is_radian
:param return_is_radian: the returned value is in radians or not, default is self.default_is_radian
:return: tuple((code, angles)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    angles: [angle-1(rad or °), angle-2, ..., angle-(Number of axes)] or []
        Note: the returned angle value is radians if return_is_radian is True, else °
```

####  1.2.18. <a name='def__get_is_moving__self:'></a>def __get_is_moving__(self):

```
Check xArm is moving or not
:return: True/False
```

####  1.2.19. <a name='def__get_position__selfis_radianNone:'></a>def __get_position__(self, is_radian=None):

```
Get the cartesian position
Note:
    1. If the value(roll/pitch/yaw) you want to return is an radian unit, please set the parameter is_radian to True
        ex: code, pos = xarm.get_position(is_radian=True)

:param is_radian: the returned value (only roll/pitch/yaw) is in radians or not, default is self.default_is_radian
:return: tuple((code, [x, y, z, roll, pitch, yaw])), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.20. <a name='def__get_servo_angle__selfservo_idNoneis_radianNone:'></a>def __get_servo_angle__(self, servo_id=None, is_radian=None):

```
Get the servo angle
Note:
    1. If the value you want to return is an radian unit, please set the parameter is_radian to True
        ex: code, angles = xarm.get_servo_angle(is_radian=True)
    2. If you want to return only the angle of a single joint, please set the parameter servo_id
        ex: code, angle = xarm.get_servo_angle(servo_id=2)

:param servo_id: 1-(Number of axes), None(8), default is None
:param is_radian: the returned value is in radians or not, default is self.default_is_radian
:return: tuple((code, angle list if servo_id is None or 8 else angle)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.21. <a name='def__get_servo_debug_msg__selfshowFalse:'></a>def __get_servo_debug_msg__(self, show=False):

```
Get the servo debug msg, used only for debugging

:param show: show the detail info if True
:return: tuple((code, servo_info_list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.22. <a name='def__get_state__self:'></a>def __get_state__(self):

```
Get state

:return: tuple((code, state)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    state:
        1: in motion
        2: sleeping
        3: suspended
        4: stopping
```

####  1.2.23. <a name='def__get_tgpio_analog__selfionumNone:'></a>def __get_tgpio_analog__(self, ionum=None):

```
Get the analog value of the specified Tool GPIO
:param ionum: 0 or 1 or None(both 0 and 1), default is None
:return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.24. <a name='def__get_tgpio_digital__selfionumNone:'></a>def __get_tgpio_digital__(self, ionum=None):

```
Get the digital value of the specified Tool GPIO

:param ionum: 0 or 1 or None(both 0 and 1), default is None
:return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.25. <a name='def__get_version__self:'></a>def __get_version__(self):

```
Get the xArm version

:return: tuple((code, version)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

####  1.2.26. <a name='def__is_joint_limit__selfjointis_radianNone:'></a>def __is_joint_limit__(self, joint, is_radian=None):

```
Check the joint is in limit

:param joint: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm
:param is_radian: angle value is radians or not, default is self.default_is_radian
:return: tuple((code, limit)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    limit: True/False/None, limit or not, or failed
```

####  1.2.27. <a name='def__is_tcp_limit__selfposeis_radianNone:'></a>def __is_tcp_limit__(self, pose, is_radian=None):

```
Check the tcp pose is in limit

:param pose: [x, y, z, roll, pitch, yaw]
:param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian
:return: tuple((code, limit)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    limit: True/False/None, limit or not, or failed
```

####  1.2.28. <a name='def__motion_enable__selfenableTrueservo_idNone:'></a>def __motion_enable__(self, enable=True, servo_id=None):

```
Motion enable

:param enable: 
:param servo_id: 1-(Number of axes), None(8)
:return: code
    code: See the API code documentation for details.
```

####  1.2.29. <a name='def__move_arc_lines__selfpathsis_radianNonetimes1first_pause_time0.1repeat_pause_time0automatic_calibrationTruespeedNonemvaccNonemvtimeNonewaitFalse:'></a>def __move_arc_lines__(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0, automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):

```
Continuous linear motion with interpolation
Note:
    1. If an error occurs, it will return early
    2. If the emergency_stop interface is called actively, it will return early.
    3. The last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified
    4. The last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified

:param paths: cartesian path list
    1. Specify arc radius： [[x, y, z, roll, pitch, yaw, radius], ....]
    1. Do not specify arc radius (radius=0)： [[x, y, z, roll, pitch, yaw], ....]
:param is_radian: roll/pitch/yaw of paths are in radians or not, default is self.default_is_radian
:param times: repeat times, 0 is infinite loop, default is 1
:param first_pause_time: sleep time at first, purpose is to cache the instruction, default is 0.1s
:param repeat_pause_time: interval between repeated movements, unit: second
:param automatic_calibration: automatic calibration or not, default is True
:param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
:param mvtime: 0, reserved 
:param wait: whether to wait for the arm to complete, default is False
```

####  1.2.30. <a name='def__move_circle__selfpose1pose2percentspeedNonemvaccNonemvtimeNoneis_radianNonewaitFalsetimeoutNonekwargs:'></a>def __move_circle__(self, pose1, pose2, percent, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):

```
The motion calculates the trajectory of the space circle according to the three-point coordinates.
The three-point coordinates are (current starting point, pose1, pose2).

:param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
:param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
:param percent: the percentage of arc length and circumference of the movement
:param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
:param mvtime: 0, reserved
:param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
        code < 0: the last_used_tcp_speed/last_used_tcp_acc will not be modified
        code >= 0: the last_used_tcp_speed/last_used_tcp_acc will be modified
```

####  1.2.31. <a name='def__move_gohome__selfspeedNonemvaccNonemvtimeNoneis_radianNonewaitFalsetimeoutNone:'></a>def __move_gohome__(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):

```
Move to go home (Back to zero), the API will modify self.last_used_position and self.last_used_angles value
Warnning: without limit detection
Note:
    1. The API will change self.last_used_position value into [201.5, 0, 140.5, -180, 0, 0]
    2. The API will change self.last_used_angles value into [0, 0, 0, 0, 0, 0, 0]
    3. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
        ex: code = xarm.move_gohome(wait=True)
    4. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc

:param speed: gohome speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s
:param mvacc: gohome acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2
:param mvtime: reserved
:param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
:return: code
    code: See the API code documentation for details.
```

####  1.2.32. <a name='def__register_cmdnum_changed_callback__selfcallbackNone:'></a>def __register_cmdnum_changed_callback__(self, callback=None):

```
Register the cmdnum changed callback, only available if enable_report is True

:param callback: 
    callback data:
    {
        "cmdnum": cmdnum
    }
:return: True/False
```

####  1.2.33. <a name='def__register_connect_changed_callback__selfcallbackNone:'></a>def __register_connect_changed_callback__(self, callback=None):

```
Register the connect status changed callback

:param callback: 
    callback data:
    {
        "connected": connected,
        "reported": reported,
    }
:return: True/False
```

####  1.2.34. <a name='def__register_error_warn_changed_callback__selfcallbackNone:'></a>def __register_error_warn_changed_callback__(self, callback=None):

```
Register the error code or warn code changed callback, only available if enable_report is True

:param callback: 
    callback data:
    {
        "error_code": error_code,
        "warn_code": warn_code,
    }
:return: True/False
```

####  1.2.35. <a name='def__register_mode_changed_callback__selfcallbackNone:'></a>def __register_mode_changed_callback__(self, callback=None):

```
Register the mode changed callback, only available if enable_report is True and the connect way is socket

:param callback:
    callback data:
    {
        "mode": mode,
    }
:return: True/False
```

####  1.2.36. <a name='def__register_mtable_mtbrake_changed_callback__selfcallbackNone:'></a>def __register_mtable_mtbrake_changed_callback__(self, callback=None):

```
Register the motor enable states or motor brake states changed callback, only available if enable_report is True and the connect way is socket

:param callback: 
    callback data:
    {
        "mtable": [motor-1-motion-enable, motor-2-motion-enable, ...],
        "mtbrake": [motor-1-brake-enable, motor-1-brake-enable,...],
    }
:return: True/False
```

####  1.2.37. <a name='def__register_report_callback__selfcallbackNonereport_cartesianTruereport_jointsTruereport_stateTruereport_error_codeTruereport_warn_codeTruereport_mtableTruereport_mtbrakeTruereport_cmd_numTrue:'></a>def __register_report_callback__(self, callback=None, report_cartesian=True, report_joints=True, report_state=True, report_error_code=True, report_warn_code=True, report_mtable=True, report_mtbrake=True, report_cmd_num=True):

```
Register the report callback, only available if enable_report is True

:param callback: 
    callback data:
    {
        'cartesian': [], # if report_cartesian is True
        'joints': [], # if report_joints is True
        'error_code': 0, # if report_error_code is True
        'warn_code': 0, # if report_warn_code is True
        'state': state, # if report_state is True
        'mtbrake': mtbrake, # if report_mtbrake is True, and available if enable_report is True and the connect way is socket
        'mtable': mtable, # if report_mtable is True, and available if enable_report is True and the connect way is socket
        'cmdnum': cmdnum, # if report_cmd_num is True
    }
:param report_cartesian: report cartesian or not, default is True
:param report_joints: report joints or not, default is True
:param report_state: report state or not, default is True
:param report_error_code: report error or not, default is True
:param report_warn_code: report warn or not, default is True
:param report_mtable: report motor enable states or not, default is True
:param report_mtbrake: report motor brake states or not, default is True
:param report_cmd_num: report cmdnum or not, default is True
:return: True/False
```

####  1.2.38. <a name='def__register_report_location_callback__selfcallbackNonereport_cartesianTruereport_jointsTrue:'></a>def __register_report_location_callback__(self, callback=None, report_cartesian=True, report_joints=True):

```
Register the report location callback, only available if enable_report is True

:param callback: 
    callback data:
    {
        "cartesian": [x, y, z, roll, pitch, yaw], ## if report_cartesian is True
        "joints": [angle-1, angle-2, angle-3, angle-4, angle-5, angle-6, angle-7], ## if report_joints is True
    }
:param report_cartesian: report or not, True/False, default is True
:param report_joints: report or not, True/False, default is True
:return: True/False
```

####  1.2.39. <a name='def__register_state_changed_callback__selfcallbackNone:'></a>def __register_state_changed_callback__(self, callback=None):

```
Register the state status changed callback, only available if enable_report is True

:param callback:
    callback data:
    {
        "state": state,
    }
:return: True/False
```

####  1.2.40. <a name='def__release_cmdnum_changed_callback__selfcallbackNone:'></a>def __release_cmdnum_changed_callback__(self, callback=None):

```
Release the cmdnum changed callback

:param callback: 
:return: True/False
```

####  1.2.41. <a name='def__release_connect_changed_callback__selfcallbackNone:'></a>def __release_connect_changed_callback__(self, callback=None):

```
Release the connect changed callback

:param callback: 
:return: True/False
```

####  1.2.42. <a name='def__release_error_warn_changed_callback__selfcallbackNone:'></a>def __release_error_warn_changed_callback__(self, callback=None):

```
Release the error warn changed callback

:param callback: 
:return: True/False
```

####  1.2.43. <a name='def__release_mode_changed_callback__selfcallbackNone:'></a>def __release_mode_changed_callback__(self, callback=None):

```
Release the mode changed callback

:param callback: 
:return: True/False
```

####  1.2.44. <a name='def__release_mtable_mtbrake_changed_callback__selfcallbackNone:'></a>def __release_mtable_mtbrake_changed_callback__(self, callback=None):

```
Release the motor enable states or motor brake states changed callback

:param callback: 
:return: True/False
```

####  1.2.45. <a name='def__release_report_callback__selfcallbackNone:'></a>def __release_report_callback__(self, callback=None):

```
Release the report callback

:param callback: 
:return: True/False
```

####  1.2.46. <a name='def__release_report_location_callback__selfcallbackNone:'></a>def __release_report_location_callback__(self, callback=None):

```
Release the location report callback

:param callback: 
:return: True/False
```

####  1.2.47. <a name='def__release_state_changed_callback__selfcallbackNone:'></a>def __release_state_changed_callback__(self, callback=None):

```
Release the state changed callback

:param callback: 
:return: True/False
```

####  1.2.48. <a name='def__reset__selfspeedNonemvaccNonemvtimeNoneis_radianNonewaitFalsetimeoutNone:'></a>def __reset__(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):

```
Reset the xArm
Warnning: without limit detection
Note:
    1. The API will change self.last_used_position value into [201.5, 0, 140.5, -180, 0, 0]
    2. The API will change self.last_used_angles value into [0, 0, 0, 0, 0, 0, 0]
    3. If there are errors or warnings, this interface will clear the warnings and errors.
    4. If not ready, the api will auto enable motion and set state
    5. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc

:param speed: reset speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s
:param mvacc: reset acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2
:param mvtime: reserved
:param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
```

####  1.2.49. <a name='def__run_blockly_app__selfpath:'></a>def __run_blockly_app__(self, path):

```
Run the app generated by xArmStudio software
:param path: app path
```

####  1.2.50. <a name='def__save_conf__self:'></a>def __save_conf__(self):

```
Save config
Note:
    1. This interface can record the current settings and will not be lost after the restart.
    2. The clean_conf interface can restore system default settings

:return: code
    code: See the API code documentation for details.
```

####  1.2.51. <a name='def__send_cmd_sync__selfcommandNone:'></a>def __send_cmd_sync__(self, command=None):

```
Send cmd and wait (only waiting the cmd response, not waiting for the movement)
Note:
    1. Some command depends on self.default_is_radian

:param command: 
    'G1': 'set_position(MoveLine): G1 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)} F{speed(mm/s)} Q{acc(mm/s^2)} T{mvtime} W{wait}'
    'G4': 'set_pause_time: G4 V{sltime(second)}'
    'G7': 'set_servo_angle: G7 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)} F{speed(°/s or rad/s)} Q{acc(°/s^2 or rad/s^2)} T{mvtime} W{wait}'
    'G8': 'move_gohome: G8 F{speed(°/s or rad/s)} Q{acc(°/s^2 or rad/s^2)} T{mvtime} W{wait}'
    'G9': 'set_position(MoveArcLine): G9 X{x} Y{y} Z{z} A{roll} B{pitch(° or rad)} C{yaw(° or rad)} R{radius(mm)} F{speed(mm/s)} Q{acc(mm/s^2)} T{mvtime} W{wait}'
    'H1': 'get_version: H1'
    'H11': 'motion_enable: H11 S{servo_id} V{enable}'
    'H12': 'set_state: H12 V{state}'
    'H13': 'get_state: H13'
    'H14': 'get_cmdnum: H14'
    'H15': 'get_err_warn_code: H15'
    'H16': 'clean_error: H16'
    'H17': 'clean_warn: H17'
    'H18': 'set_servo_attach/set_servo_detach: H18 S{servo_id} V{1: enable(detach), 0: disable(attach)}'
    'H31': 'set_tcp_jerk: H31 V{jerk(mm/s^3)}'
    'H32': 'set_tcp_maxacc: H32 V{maxacc(mm/s^2)}'
    'H33': 'set_joint_jerk: H33 V{jerk(°/s^3 or rad/s^3)}'
    'H34': 'set_joint_maxacc: H34 {maxacc(°/s^2 or rad/s^2)}'
    'H39': 'clean_conf: H39'
    'H40': 'save_conf: H40'
    'H41': 'get_position: H41'
    'H42': 'get_servo_angle: H42'
    'H43': 'get_inverse_kinematics: H43 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)}'
    'H44': 'get_forward_kinematics: H44 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)}'
    'H45': 'is_joint_limit: H45 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)}'
    'H46': 'is_tcp_limit: H46 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)}'
    'H106': 'get_servo_debug_msg: H106'
:return: code or tuple((code, ...))
    code: See the API code documentation for details.
```

####  1.2.52. <a name='def__set_cgpio_analog__selfionumvalue:'></a>def __set_cgpio_analog__(self, ionum, value):

```
Set the analog value of the specified Controller GPIO

:param ionum: 0 or 1
:param value: value
:return: code
    code: See the API code documentation for details.
```

####  1.2.53. <a name='def__set_cgpio_digital__selfionumvalue:'></a>def __set_cgpio_digital__(self, ionum, value):

```
Set the digital value of the specified Controller GPIO

:param ionum: 0~7
:param value: value
:return: code
    code: See the API code documentation for details.
```

####  1.2.54. <a name='def__set_cgpio_digital_input_function__selfionumfun:'></a>def __set_cgpio_digital_input_function__(self, ionum, fun):

```
Set the digital input functional mode of the Controller GPIO
:param ionum: 0~7
:param fun: functional mode
:return: code
    code: See the API code documentation for details.
```

####  1.2.55. <a name='def__set_cgpio_digital_output_function__selfionumfun:'></a>def __set_cgpio_digital_output_function__(self, ionum, fun):

```
Set the digital output functional mode of the specified Controller GPIO
:param ionum: 0~7
:param fun: functionnal mode
    0: system in stopping
    1: controller has error
    2: in motion
:return: code
    code: See the API code documentation for details.
```

####  1.2.56. <a name='def__set_collision_sensitivity__selfvalue:'></a>def __set_collision_sensitivity__(self, value):

```
Set the sensitivity of collision

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param value: sensitivity value, 0~5
:return: code
    code: See the API code documentation for details.
```

####  1.2.57. <a name='def__set_gravity_direction__selfdirection:'></a>def __set_gravity_direction__(self, direction):

```
Set the direction of gravity

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param direction: direction of gravity, such as [x(mm), y(mm), z(mm)]
:return: code
    code: See the API code documentation for details.
```

####  1.2.58. <a name='def__set_gripper_enable__selfenable:'></a>def __set_gripper_enable__(self, enable):

```
Set the gripper enable

:param enable: 
:return: code
    code: See the Gripper code documentation for details.
```

####  1.2.59. <a name='def__set_gripper_mode__selfmode:'></a>def __set_gripper_mode__(self, mode):

```
Set the gripper mode

:param mode: 1: location mode, 2: speed mode (no use), 3: torque mode (no use)
:return: code
    code: See the Gripper code documentation for details.
```

####  1.2.60. <a name='def__set_gripper_position__selfposwaitFalsespeedNoneauto_enableFalsetimeoutNone:'></a>def __set_gripper_position__(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):

```
Set the gripper position

:param pos: pos
:param wait: wait or not, default is False
:param speed: speed
:param auto_enable: auto enable or not, default is False
:param timeout: second, default is 10s
:return: code
    code: See the Gripper code documentation for details.
```

####  1.2.61. <a name='def__set_gripper_speed__selfspeed:'></a>def __set_gripper_speed__(self, speed):

```
Set the gripper speed

:param speed: 
:return: code
    code: See the Gripper code documentation for details.
```

####  1.2.62. <a name='def__set_joint_jerk__selfjerkis_radianNone:'></a>def __set_joint_jerk__(self, jerk, is_radian=None):

```
Set the jerk of Joint space
Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param jerk: jerk (°/s^3 or rad/s^3)
:param is_radian: the jerk in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

####  1.2.63. <a name='def__set_joint_maxacc__selfaccis_radianNone:'></a>def __set_joint_maxacc__(self, acc, is_radian=None):

```
Set the max acceleration of Joint space

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param acc: max acceleration (°/s^2 or rad/s^2)
:param is_radian: the jerk in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

####  1.2.64. <a name='def__set_mode__selfmode0:'></a>def __set_mode__(self, mode=0):

```
Set the xArm mode

:param mode: default is 0
    0: position control mode
    1: servo motion mode
        Note: the use of the set_servo_angle_j interface must first be set to this mode 
    2: joint teaching mode
        Note: use this mode to ensure that the arm has been identified and the control box and arm used for identification are one-to-one.
    3: cartesian teaching mode (invalid)
:return: code
    code: See the API code documentation for details.
```

####  1.2.65. <a name='def__set_mount_direction__selfbase_tilt_degrotation_degis_radianNone:'></a>def __set_mount_direction__(self, base_tilt_deg, rotation_deg, is_radian=None):

```
Set the mount direction

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings
    
:param base_tilt_deg: tilt degree
:param rotation_deg: rotation degree
:param is_radian: the jebase_tilt_deg/rotation_deg in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

####  1.2.66. <a name='def__set_pause_time__selfsltimewaitFalse:'></a>def __set_pause_time__(self, sltime, wait=False):

```
Set the arm pause time, xArm will pause sltime second

:param sltime: sleep second
:param wait: wait or not, default is False
:return: code
    code: See the API code documentation for details.
```

####  1.2.67. <a name='def__set_position__selfxNoneyNonezNonerollNonepitchNoneyawNoneradiusNonespeedNonemvaccNonemvtimeNonerelativeFalseis_radianNonewaitFalsetimeoutNonekwargs:'></a>def __set_position__(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, **kwargs):

```
Set the cartesian position, the API will modify self.last_used_position value
Note:
    1. If it is a 5-axis arm, ensure that the current robot arm has a roll value of 180° or π rad and has a roll value of 0 before calling this interface.
    2. If it is a 5-axis arm, roll must be set to 180° or π rad, pitch must be set to 0
    3. If the parameter(roll/pitch/yaw) you are passing is an radian unit, be sure to set the parameter is_radian to True.
        ex: code = xarm.set_position(x=300, y=0, z=200, roll=-3.14, pitch=0, yaw=0, is_radian=True)
    4. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
        ex: code = xarm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, is_radian=False, wait=True)

:param x: cartesian position x, (unit: mm), default is self.last_used_position[0]
:param y: cartesian position y, (unit: mm), default is self.last_used_position[1]
:param z: cartesian position z, (unit: mm), default is self.last_used_position[2]
:param roll: rotate around the X axis, (unit: rad if is_radian is True else °), default is self.last_used_position[3]
:param pitch: rotate around the Y axis, (unit: rad if is_radian is True else °), default is self.last_used_position[4]
:param yaw: rotate around the Z axis, (unit: rad if is_radian is True else °), default is self.last_used_position[5]
:param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
    MoveLine: Linear motion
        ex: code = xarm.set_position(..., radius=None)
    MoveArcLine: Linear arc motion with interpolation
        ex: code = xarm.set_position(..., radius=0)
:param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
:param mvtime: 0, reserved
:param relative: relative move or not
:param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
        code < 0: the last_used_position/last_used_tcp_speed/last_used_tcp_acc will not be modified
        code >= 0: the last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified
```

####  1.2.68. <a name='def__set_servo_angle__selfservo_idNoneangleNonespeedNonemvaccNonemvtimeNonerelativeFalseis_radianNonewaitFalsetimeoutNonekwargs:'></a>def __set_servo_angle__(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, **kwargs):

```
Set the servo angle, the API will modify self.last_used_angles value
Note:
    1. If the parameter angle you are passing is an radian unit, be sure to set the parameter is_radian to True.
        ex: code = xarm.set_servo_angle(servo_id=1, angle=1.57, is_radian=True)
    2. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
        ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False,wait=True)

:param servo_id: 1-(Number of axes), None(8)
    1. 1-(Number of axes) indicates the corresponding joint, the parameter angle should be a numeric value
        ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False)
    2. None(8) means all joints, default is None, the parameter angle should be a list of values whose length is the number of joints
        ex: code = xarm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 0], is_radian=False)
:param angle: angle or angle list, (unit: rad if is_radian is True else °)
    1. If servo_id is 1-(Number of axes), angle should be a numeric value
        ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False)
    2. If servo_id is None or 8, angle should be a list of values whose length is the number of joints
        like [axis-1, axis-2, axis-3, axis-3, axis-4, axis-5, axis-6, axis-7]
        ex: code = xarm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 0], is_radian=False)
:param speed: move speed (unit: rad/s if is_radian is True else °/s), default is self.last_used_joint_speed
:param mvacc: move acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is self.last_used_joint_acc
:param mvtime: 0, reserved
:param relative: relative move or not
:param is_radian: the angle in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
        code < 0: the last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified
        code >= 0: the last_used_angles/last_used_joint_speed/last_used_joint_acc will be modified
```

####  1.2.69. <a name='def__set_servo_angle_j__selfanglesspeedNonemvaccNonemvtimeNoneis_radianNonekwargs:'></a>def __set_servo_angle_j__(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):

```
Set the servo angle, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))
Note:
    1. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc

:param angles: angle list, (unit: rad if is_radian is True else °)
:param speed: speed, reserved
:param mvacc: acceleration, reserved
:param mvtime: 0, reserved
:param is_radian: the angles in radians or not, defalut is self.default_is_radian
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
```

####  1.2.70. <a name='def__set_servo_attach__selfservo_idNone:'></a>def __set_servo_attach__(self, servo_id=None):

```
Attach the servo

:param servo_id: 1-(Number of axes), 8, if servo_id is 8, will attach all servo
    1. 1-(Number of axes): attach only one joint
        ex: xarm.set_servo_attach(servo_id=1)
    2: 8: attach all joints
        ex: xarm.set_servo_attach(servo_id=8)
:return: code
    code: See the API code documentation for details.
```

####  1.2.71. <a name='def__set_servo_detach__selfservo_idNone:'></a>def __set_servo_detach__(self, servo_id=None):

```
Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.

:param servo_id: 1-(Number of axes), 8, if servo_id is 8, will detach all servo
    1. 1-(Number of axes): detach only one joint
        ex: xarm.set_servo_detach(servo_id=1)
    2: 8: detach all joints, please
        ex: xarm.set_servo_detach(servo_id=8)
:return: code
    code: See the API code documentation for details.
```

####  1.2.72. <a name='def__set_state__selfstate0:'></a>def __set_state__(self, state=0):

```
Set the xArm state

:param state: default is 0
    0: sport state
    3: pause state
    4: stop state
:return: code
    code: See the API code documentation for details.
```

####  1.2.73. <a name='def__set_tcp_jerk__selfjerk:'></a>def __set_tcp_jerk__(self, jerk):

```
Set the translational jerk of Cartesian space
Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param jerk: jerk (mm/s^3)
:return: code
    code: See the API code documentation for details.
```

####  1.2.74. <a name='def__set_tcp_load__selfweightcenter_of_gravity:'></a>def __set_tcp_load__(self, weight, center_of_gravity):

```
Set the load

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param weight: load weight (unit: kg)
:param center_of_gravity: load center of gravity, such as [x(mm), y(mm), z(mm)]
:return: code
    code: See the API code documentation for details.
```

####  1.2.75. <a name='def__set_tcp_maxacc__selfacc:'></a>def __set_tcp_maxacc__(self, acc):

```
Set the max translational acceleration of Cartesian space
Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param acc: max acceleration (mm/s^2)
:return: code
    code: See the API code documentation for details.
```

####  1.2.76. <a name='def__set_tcp_offset__selfoffsetis_radianNone:'></a>def __set_tcp_offset__(self, offset, is_radian=None):

```
Set the tool coordinate system offset at the end
Note:
    1. Do not use if not required
    2. If not saved and you want to revert to the last saved value, please reset the offset by set_tcp_offset([0, 0, 0, 0, 0, 0])
    3. If not saved, it will be lost after reboot
    4. The save_conf interface can record the current settings and will not be lost after the restart.
    5. The clean_conf interface can restore system default settings

:param offset: [x, y, z, roll, pitch, yaw]
:param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

####  1.2.77. <a name='def__set_teach_sensitivity__selfvalue:'></a>def __set_teach_sensitivity__(self, value):

```
Set the sensitivity of drag and teach

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param value: sensitivity value, 1~5
:return: code
    code: See the API code documentation for details.
```

####  1.2.78. <a name='def__set_tgpio_digital__selfionumvalue:'></a>def __set_tgpio_digital__(self, ionum, value):

```
Set the digital value of the specified Tool GPIO

:param ionum: 0 or 1
:param value: value
:return: code
    code: See the API code documentation for details.
```

####  1.2.79. <a name='def__shutdown_system__selfvalue1:'></a>def __shutdown_system__(self, value=1):

```
Shutdown the xArm controller system

:param value: 1: remote shutdown
:return: code
    code: See the API code documentation for details.
```
