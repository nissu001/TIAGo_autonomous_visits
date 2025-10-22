cmake_minimum_required(VERSION 2.8.3)
project(pal_interaction_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

# Mensajes
add_message_files(
  FILES
    actiontag.msg
    ASRActivation.msg
    ASREvent.msg
    ASRLangModelMngmt.msg
    ASRLanguage.msg
    asrresult.msg
    ASRSrvRequest.msg
    ASRSrvResponse.msg
    ASRStatus.msg
    asrupdate.msg
    AudioDeviceDescription.msg
    AudioPlayerState.msg
    audiosignal.msg
    DirectionOfArrival.msg
    EnablingSoundLocalisation.msg
    I18nText.msg
    I18nArgument.msg
    Input.msg
    InputArgument.msg
    TTSstate.msg
    TtsText.msg
    TtsMark.msg
    ##TtsActionGoal.msg
    VoiceActivity.msg
    WebGuiEvent.msg
)

# Acciones
add_action_files(
  FILES
    ASRFile.action
    AudioPlay.action
    Sound.action
    Tts.action
)

# Servicios
add_service_files(
  FILES
    ASRService.srv
    GetSpeechDuration.srv
    recognizerService.srv
    SoundLocalisationService.srv
)

# Generar mensajes y servicios
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

# Declarar el paquete catkin
catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

