//---------------------------------------------------------------------------*
//                                                                           *
//                       File 'goil_target_c166.cpp'                         *
//                        Generated by version 1.8.5                         *
//                      june 19th, 2009, at 17h48'26"                        *
//                                                                           *
//---------------------------------------------------------------------------*

//--- START OF USER ZONE 1


//--- END OF USER ZONE 1

//---------------------------------------------------------------------------*

#include "version_libpm.h"
#if LIBPM_VERSION != 515
  #error "This file has been compiled with a version of GALGAS different than the version of libpm"
#endif

//---------------------------------------------------------------------------*

#include <typeinfo>
#include "utilities/MF_MemoryControl.h"
#include "files/C_TextFileWrite.h"
#include "goil_target_c166.h"

//---------------------------------------------------------------------------*

#ifndef DO_NOT_GENERATE_CHECKINGS
  #define SOURCE_FILE_AT_LINE(line) "goil_target_c166.ggs", line
  #define COMMA_SOURCE_FILE_AT_LINE(line) , SOURCE_FILE_AT_LINE(line)
#else
  #define SOURCE_FILE_AT_LINE(line) 
  #define COMMA_SOURCE_FILE_AT_LINE(line) 
#endif


//--- START OF USER ZONE 2


//--- END OF USER ZONE 2

//---------------------------------------------------------------------------*
//                                                                           *
//             Implementation of routine "generate_target_c166"              *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_generate_target_c166 (C_Compiler & inLexique,
                                GGS_lstring   var_cas_name,
                                GGS_ident_map   var_cas_others,
                                GGS_oil_obj   var_cas_exe,
                                GGS_string & var_cas_result COMMA_UNUSED_LOCATION_ARGS) {
  #ifdef DEBUG_TRACE_ENABLED
    printf ("ENTER routine_generate_target_c166 at %s:%d\n", __FILE__, __LINE__) ;
  #endif
  ::routine_doReplace (inLexique,  var_cas_result,  GGS_string ("$SYS_STACK_ZONE$"),  (var_cas_name.reader_string (inLexique COMMA_SOURCE_FILE_AT_LINE (14))).operator_concat (GGS_string ("_sys_stack")) COMMA_SOURCE_FILE_AT_LINE (14)) ;
  ::routine_doReplace (inLexique,  var_cas_result,  GGS_string ("$USR_STACK_ZONE$"),  (var_cas_name.reader_string (inLexique COMMA_SOURCE_FILE_AT_LINE (15))).operator_concat (GGS_string ("_usr_stack")) COMMA_SOURCE_FILE_AT_LINE (15)) ;
  ::routine_doReplace (inLexique,  var_cas_result,  GGS_string ("$EXEC_INTEGER_CONTEXT$"),  (var_cas_name.reader_string (inLexique COMMA_SOURCE_FILE_AT_LINE (16))).operator_concat (GGS_string ("_int_context")) COMMA_SOURCE_FILE_AT_LINE (16)) ;
  GGS_uint  var_cas_usr_stack_size ;
  GGS_uint  var_cas_sys_stack_size ;
  ::routine_additional_int_key_required (inLexique,  GGS_string ("USRSTACKSIZE"),  var_cas_others,  var_cas_name,  var_cas_usr_stack_size COMMA_SOURCE_FILE_AT_LINE (20)) ;
  ::routine_additional_int_key_required (inLexique,  GGS_string ("SYSSTACKSIZE"),  var_cas_others,  var_cas_name,  var_cas_sys_stack_size COMMA_SOURCE_FILE_AT_LINE (21)) ;
  ::routine_doReplace (inLexique,  var_cas_result,  GGS_string ("$USR_STACK_SIZE$"),  var_cas_usr_stack_size.reader_string (inLexique COMMA_SOURCE_FILE_AT_LINE (22)) COMMA_SOURCE_FILE_AT_LINE (22)) ;
  ::routine_doReplace (inLexique,  var_cas_result,  GGS_string ("$SYS_STACK_SIZE$"),  var_cas_sys_stack_size.reader_string (inLexique COMMA_SOURCE_FILE_AT_LINE (23)) COMMA_SOURCE_FILE_AT_LINE (23)) ;
  { const GGS_oil_obj _var_1228 = var_cas_exe ; // CAST instruction
    if (_var_1228.getPtr () != NULL) {
      macroValidPointer (_var_1228.getPtr ()) ;
      if (typeid (cPtr_isr_obj) == typeid (* (_var_1228.getPtr ()))) {
        GGS_uint  var_cas_trap ;
        ::routine_additional_int_key_required (inLexique,  GGS_string ("TRAP"),  var_cas_others,  var_cas_name,  var_cas_trap COMMA_SOURCE_FILE_AT_LINE (29)) ;
        ::routine_doReplace (inLexique,  var_cas_result,  GGS_string ("$TRAP_NUMBER$"),  var_cas_trap.reader_string (inLexique COMMA_SOURCE_FILE_AT_LINE (30)) COMMA_SOURCE_FILE_AT_LINE (30)) ;
      }else{
      }
    }
  }
  #ifdef DEBUG_TRACE_ENABLED
    printf ("LEAVE routine_generate_target_c166\n") ;
  #endif
}

//---------------------------------------------------------------------------*
//                                                                           *
//              Implementation of routine "generate_isr_c166"                *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_generate_isr_c166 (C_Compiler &,
                                const GGS_isr_map   /* var_cas_isrs */,
                                GGS_string & /* var_cas_code */ COMMA_UNUSED_LOCATION_ARGS) {
  #ifdef DEBUG_TRACE_ENABLED
    printf ("ENTER routine_generate_isr_c166 at %s:%d\n", __FILE__, __LINE__) ;
  #endif
  #ifdef DEBUG_TRACE_ENABLED
    printf ("LEAVE routine_generate_isr_c166\n") ;
  #endif
}

//---------------------------------------------------------------------------*
//                                                                           *
//            Implementation of routine "generate_counter_c166"              *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_generate_counter_c166 (C_Compiler & inLexique,
                                const GGS_counter_map   var_cas_counters,
                                GGS_string & var_cas_code COMMA_UNUSED_LOCATION_ARGS) {
  #ifdef DEBUG_TRACE_ENABLED
    printf ("ENTER routine_generate_counter_c166 at %s:%d\n", __FILE__, __LINE__) ;
  #endif
  GGS_string var_cas_result ;
  var_cas_result = GGS_string ("") ;
  {
    GGS_counter_map::cEnumerator enumerator_1471 (var_cas_counters, true) ;
    const GGS_counter_map::cElement * operand_1471 = NULL ;
    while (((operand_1471 = enumerator_1471.nextObject ()))) {
      macroValidPointer (operand_1471) ;
      GGS_string var_cas_tmp ;
      ::routine_retrieveTemplateString (inLexique,  var_cas_tmp,  GGS_string ("counter_list_specific") COMMA_SOURCE_FILE_AT_LINE (48)) ;
      ::routine_doReplace (inLexique,  var_cas_tmp,  GGS_string ("$COUNTER$"),  (operand_1471->mKey.reader_string (inLexique COMMA_SOURCE_FILE_AT_LINE (49))).operator_concat (GGS_string ("_counter_desc")) COMMA_SOURCE_FILE_AT_LINE (49)) ;
      var_cas_result.dotAssign_operation (var_cas_tmp) ;
    }
  }
  ::routine_doReplace (inLexique,  var_cas_code,  GGS_string ("$COUNTER_LIST$"),  var_cas_result COMMA_SOURCE_FILE_AT_LINE (53)) ;
  #ifdef DEBUG_TRACE_ENABLED
    printf ("LEAVE routine_generate_counter_c166\n") ;
  #endif
}

//---------------------------------------------------------------------------*

