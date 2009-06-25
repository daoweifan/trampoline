//---------------------------------------------------------------------------*
//                                                                           *
//                         File 'goil_targets.cpp'                           *
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
#include "goil_targets.h"

//---------------------------------------------------------------------------*

#ifndef DO_NOT_GENERATE_CHECKINGS
  #define SOURCE_FILE_AT_LINE(line) "goil_targets.ggs", line
  #define COMMA_SOURCE_FILE_AT_LINE(line) , SOURCE_FILE_AT_LINE(line)
#else
  #define SOURCE_FILE_AT_LINE(line) 
  #define COMMA_SOURCE_FILE_AT_LINE(line) 
#endif


//--- START OF USER ZONE 2


//--- END OF USER ZONE 2

//---------------------------------------------------------------------------*
//                                                                           *
//          Implementation of routine "generate_context_and_stack"           *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_generate_context_and_stack (C_Compiler & inLexique,
                                GGS_config   var_cas_cfg,
                                GGS_lstring   var_cas_name,
                                GGS_oil_obj   var_cas_exe,
                                GGS_string & var_cas_result COMMA_UNUSED_LOCATION_ARGS) {
  #ifdef DEBUG_TRACE_ENABLED
    printf ("ENTER routine_generate_context_and_stack at %s:%d\n", __FILE__, __LINE__) ;
  #endif
  GGS_string var_cas_tpl_context_and_stack ;
  var_cas_tpl_context_and_stack = GGS_string ("") ;
  GGS_string var_cas_context_and_stack ;
  var_cas_context_and_stack = GGS_string ("") ;
  GGS_ident_map  var_cas_idents ;
  var_cas_idents = GGS_ident_map ::constructor_emptyMap (inLexique COMMA_HERE) ;
  GGS_attrs  var_cas_attrs ;
  var_cas_attrs = GGS_attrs ::constructor_new (inLexique, GGS_location (inLexique), GGS_cfg_attrs ::constructor_emptyMap (inLexique COMMA_HERE) COMMA_HERE) ;
  GGS_string var_cas_target ;
  var_cas_target = function_arch (inLexique COMMA_SOURCE_FILE_AT_LINE (62)) ;
  { const GGS_oil_obj _var_3236 = var_cas_exe ; // CAST instruction
    if (_var_3236.getPtr () != NULL) {
      macroValidPointer (_var_3236.getPtr ()) ;
      if (typeid (cPtr_task_obj) == typeid (* (_var_3236.getPtr ()))) {
        const GGS_task_obj var_cas_tk (_var_3236.getPtr ()) ;
        ::routine_retrieveTemplateString (inLexique,  var_cas_tpl_context_and_stack,  GGS_string ("each_task_specific") COMMA_SOURCE_FILE_AT_LINE (66)) ;
        var_cas_context_and_stack = var_cas_tpl_context_and_stack ;
        var_cas_idents = var_cas_tk.reader_others (inLexique COMMA_SOURCE_FILE_AT_LINE (68)) ;
        GGS_basic_type  var_cas_b_attrs ;
        const GGS_config  temp_2519 = var_cas_cfg ;
        if (temp_2519.isBuilt ()) {
          temp_2519 (HERE)->method_get (inLexique, GGS_lstring ::constructor_new (inLexique, GGS_string ("task_attrs"), GGS_location (inLexique) COMMA_HERE), var_cas_b_attrs COMMA_SOURCE_FILE_AT_LINE (70)) ;
        }
        { const GGS_basic_type _var_2719 = var_cas_b_attrs ; // CAST instruction
          if (_var_2719.getPtr () != NULL) {
            macroValidPointer (_var_2719.getPtr ()) ;
            if (typeid (cPtr_attrs) == typeid (* (_var_2719.getPtr ()))) {
              const GGS_attrs var_cas_at (_var_2719.getPtr ()) ;
              var_cas_attrs = var_cas_at ;
            }else{
              GGS_location (inLexique).reader_location (inLexique COMMA_HERE).signalGGSSemanticError (inLexique, GGS_string ("Internal error, unknown config task attributes") COMMA_SOURCE_FILE_AT_LINE (74)) ;
            }
          }
        }
      }else if (typeid (cPtr_isr_obj) == typeid (* (_var_3236.getPtr ()))) {
        const GGS_isr_obj var_cas_isr (_var_3236.getPtr ()) ;
        ::routine_retrieveTemplateString (inLexique,  var_cas_tpl_context_and_stack,  GGS_string ("each_isr_specific") COMMA_SOURCE_FILE_AT_LINE (76)) ;
        var_cas_context_and_stack = var_cas_tpl_context_and_stack ;
        var_cas_idents = var_cas_isr.reader_other_fields (inLexique COMMA_SOURCE_FILE_AT_LINE (78)) ;
        GGS_basic_type  var_cas_b_attrs ;
        const GGS_config  temp_2965 = var_cas_cfg ;
        if (temp_2965.isBuilt ()) {
          temp_2965 (HERE)->method_get (inLexique, GGS_lstring ::constructor_new (inLexique, GGS_string ("isr_attrs"), GGS_location (inLexique) COMMA_HERE), var_cas_b_attrs COMMA_SOURCE_FILE_AT_LINE (80)) ;
        }
        { const GGS_basic_type _var_3163 = var_cas_b_attrs ; // CAST instruction
          if (_var_3163.getPtr () != NULL) {
            macroValidPointer (_var_3163.getPtr ()) ;
            if (typeid (cPtr_attrs) == typeid (* (_var_3163.getPtr ()))) {
              const GGS_attrs var_cas_at (_var_3163.getPtr ()) ;
              var_cas_attrs = var_cas_at ;
            }else{
              GGS_location (inLexique).reader_location (inLexique COMMA_HERE).signalGGSSemanticError (inLexique, GGS_string ("Internal error, unknown config isr attributes") COMMA_SOURCE_FILE_AT_LINE (84)) ;
            }
          }
        }
      }else{
        var_cas_name.reader_location (inLexique COMMA_HERE).signalGGSSemanticError (inLexique, GGS_string ("Unknown executable object") COMMA_SOURCE_FILE_AT_LINE (87)) ;
      }
    }
  }
  if (((var_cas_target).operator_isEqual (GGS_string ("c166"))).isBuiltAndTrue ()) {
    ::routine_generate_target_c166 (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (90)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("libpcl"))).isBuiltAndTrue ()) {
    ::routine_generate_target_libpcl (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (92)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("posix"))).isBuiltAndTrue ()) {
    ::routine_generate_target_posix (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (94)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("posix_vp2"))).isBuiltAndTrue ()) {
    ::routine_generate_target_posix_vp2 (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (96)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("hcs12"))).isBuiltAndTrue ()) {
    ::routine_generate_target_hcs12 (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (98)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("ppc"))).isBuiltAndTrue ()) {
    ::routine_generate_target_ppc (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (100)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("v850e"))).isBuiltAndTrue ()) {
    ::routine_generate_target_v850e (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (102)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("h8300h"))).isBuiltAndTrue ()) {
    ::routine_generate_target_h8300h (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (104)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("arm"))).isBuiltAndTrue ()) {
    ::routine_generate_target_arm7 (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (106)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("avr"))).isBuiltAndTrue ()) {
    ::routine_generate_target_avr (inLexique,  var_cas_name,  var_cas_idents,  var_cas_exe,  var_cas_context_and_stack COMMA_SOURCE_FILE_AT_LINE (108)) ;
  }else{
    GGS_location (inLexique).reader_location (inLexique COMMA_HERE).signalGGSSemanticError (inLexique, (GGS_string ("unknown target: ")).operator_concat (var_cas_target) COMMA_SOURCE_FILE_AT_LINE (111)) ;
  }
  var_cas_result = (var_cas_result).operator_concat (var_cas_context_and_stack) ;
  #ifdef DEBUG_TRACE_ENABLED
    printf ("LEAVE routine_generate_context_and_stack\n") ;
  #endif
}

//---------------------------------------------------------------------------*
//                                                                           *
//            Implementation of routine "generate_isr_specific"              *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_generate_isr_specific (C_Compiler & inLexique,
                                const GGS_isr_map   var_cas_isrs,
                                GGS_string & var_cas_code COMMA_UNUSED_LOCATION_ARGS) {
  #ifdef DEBUG_TRACE_ENABLED
    printf ("ENTER routine_generate_isr_specific at %s:%d\n", __FILE__, __LINE__) ;
  #endif
  GGS_string var_cas_target ;
  var_cas_target = function_arch (inLexique COMMA_SOURCE_FILE_AT_LINE (120)) ;
  ::routine_retrieveTemplateString (inLexique,  var_cas_code,  GGS_string ("isr_specific") COMMA_SOURCE_FILE_AT_LINE (122)) ;
  if (((var_cas_target).operator_isEqual (GGS_string ("c166"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_c166 (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (125)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("libpcl"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_libpcl (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (127)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("posix"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_posix (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (129)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("posix_vp2"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_posix_vp2 (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (131)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("hcs12"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_hcs12 (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (133)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("ppc"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_ppc (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (135)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("v850e"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_v850e (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (137)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("h8300h"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_h8300h (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (139)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("arm"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_arm7 (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (141)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("avr"))).isBuiltAndTrue ()) {
    ::routine_generate_isr_avr (inLexique,  var_cas_isrs,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (143)) ;
  }else{
    GGS_location (inLexique).reader_location (inLexique COMMA_HERE).signalGGSSemanticError (inLexique, (GGS_string ("unknown target: ")).operator_concat (var_cas_target) COMMA_SOURCE_FILE_AT_LINE (146)) ;
  }
  #ifdef DEBUG_TRACE_ENABLED
    printf ("LEAVE routine_generate_isr_specific\n") ;
  #endif
}

//---------------------------------------------------------------------------*
//                                                                           *
//         Implementation of routine "generate_counter_for_target"           *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_generate_counter_for_target (C_Compiler & inLexique,
                                const GGS_string  var_cas_target,
                                const GGS_counter_map   var_cas_counters,
                                GGS_string & var_cas_code COMMA_UNUSED_LOCATION_ARGS) {
  #ifdef DEBUG_TRACE_ENABLED
    printf ("ENTER routine_generate_counter_for_target at %s:%d\n", __FILE__, __LINE__) ;
  #endif
  if (((var_cas_target).operator_isEqual (GGS_string ("c166"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_c166 (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (155)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("libpcl"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_libpcl (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (157)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("posix"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_posix (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (159)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("posix_vp2"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_posix_vp2 (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (161)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("hcs12"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_hcs12 (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (163)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("ppc"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_ppc (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (165)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("v850e"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_v850e (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (167)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("h8300h"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_h8300h (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (169)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("arm"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_arm7 (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (171)) ;
  }else if (((var_cas_target).operator_isEqual (GGS_string ("avr"))).isBuiltAndTrue ()) {
    ::routine_generate_counter_avr (inLexique,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (173)) ;
  }else{
    GGS_location (inLexique).reader_location (inLexique COMMA_HERE).signalGGSSemanticError (inLexique, (GGS_string ("unknown target: ")).operator_concat (var_cas_target) COMMA_SOURCE_FILE_AT_LINE (176)) ;
  }
  #ifdef DEBUG_TRACE_ENABLED
    printf ("LEAVE routine_generate_counter_for_target\n") ;
  #endif
}

//---------------------------------------------------------------------------*
//                                                                           *
//          Implementation of routine "generate_counter_specific"            *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_generate_counter_specific (C_Compiler & inLexique,
                                const GGS_counter_map   var_cas_counters,
                                GGS_string & var_cas_code COMMA_UNUSED_LOCATION_ARGS) {
  #ifdef DEBUG_TRACE_ENABLED
    printf ("ENTER routine_generate_counter_specific at %s:%d\n", __FILE__, __LINE__) ;
  #endif
  GGS_string var_cas_target ;
  var_cas_target = function_arch (inLexique COMMA_SOURCE_FILE_AT_LINE (184)) ;
  if ((GGS_bool (inLexique.boolOptionValueFromKeys ("goil_options", "posix_it" COMMA_SOURCE_FILE_AT_LINE (186)))).isBuiltAndTrue ()) {
    GGS_counter_by_source  var_cas_cbs ;
    var_cas_cbs = GGS_counter_by_source ::constructor_emptySortedList () ;
    {
      GGS_counter_map::cEnumerator enumerator_6524 (var_cas_counters, true) ;
      const GGS_counter_map::cElement * operand_6524 = NULL ;
      while (((operand_6524 = enumerator_6524.nextObject ()))) {
        macroValidPointer (operand_6524) ;
        var_cas_cbs._addAssign_operation (operand_6524->mInfo.counter.reader_source (inLexique COMMA_SOURCE_FILE_AT_LINE (189)).reader_string (inLexique COMMA_SOURCE_FILE_AT_LINE (189)), operand_6524->mKey, operand_6524->mInfo.counter) ;
      }
    }
    GGS_string var_cas_prev_src ;
    var_cas_prev_src = GGS_string ("") ;
    GGS_counter_map  var_cas_cnt_set ;
    var_cas_cnt_set = GGS_counter_map ::constructor_emptyMap (inLexique COMMA_HERE) ;
    var_cas_code = GGS_string ("") ;
    {
      GGS_counter_by_source::cEnumerator enumerator_6696 (var_cas_cbs, true) ;
      const GGS_counter_by_source::cElement * operand_6696 = NULL ;
      while (((operand_6696 = enumerator_6696.nextObject ()))) {
        macroValidPointer (operand_6696) ;
        if (((operand_6696->source).operator_isNotEqual (var_cas_prev_src)).isBuiltAndTrue ()) {
          if (((var_cas_cnt_set.reader_count (inLexique COMMA_SOURCE_FILE_AT_LINE (196))).operator_strictSup (GGS_uint (0U))).isBuiltAndTrue ()) {
            GGS_string var_cas_cnt_set_code ;
            ::routine_retrieveTemplateString (inLexique,  var_cas_cnt_set_code,  GGS_string ("counter_specific") COMMA_SOURCE_FILE_AT_LINE (198)) ;
            ::routine_doReplace (inLexique,  var_cas_cnt_set_code,  GGS_string ("$TICK_FUNC$"),  GGS_string ("tpl_tick_") COMMA_SOURCE_FILE_AT_LINE (199)) ;
            ::routine_doReplace (inLexique,  var_cas_cnt_set_code,  GGS_string ("$IT_SOURCE$"),  var_cas_prev_src COMMA_SOURCE_FILE_AT_LINE (200)) ;
            ::routine_generate_counter_for_target (inLexique,  var_cas_target,  var_cas_cnt_set,  var_cas_cnt_set_code COMMA_SOURCE_FILE_AT_LINE (201)) ;
            var_cas_code.dotAssign_operation (var_cas_cnt_set_code) ;
            var_cas_cnt_set = GGS_counter_map ::constructor_emptyMap (inLexique COMMA_HERE) ;
          }
        }
        var_cas_cnt_set.modifier_put (inLexique, operand_6696->name, operand_6696->counter COMMA_SOURCE_FILE_AT_LINE (206)) ;
        var_cas_prev_src = operand_6696->source ;
      }
    }
    if (((var_cas_cnt_set.reader_count (inLexique COMMA_SOURCE_FILE_AT_LINE (209))).operator_strictSup (GGS_uint (0U))).isBuiltAndTrue ()) {
      GGS_string var_cas_cnt_set_code ;
      ::routine_retrieveTemplateString (inLexique,  var_cas_cnt_set_code,  GGS_string ("counter_specific") COMMA_SOURCE_FILE_AT_LINE (211)) ;
      ::routine_doReplace (inLexique,  var_cas_cnt_set_code,  GGS_string ("$TICK_FUNC$"),  GGS_string ("tpl_tick_") COMMA_SOURCE_FILE_AT_LINE (212)) ;
      ::routine_doReplace (inLexique,  var_cas_cnt_set_code,  GGS_string ("$IT_SOURCE$"),  var_cas_prev_src COMMA_SOURCE_FILE_AT_LINE (213)) ;
      ::routine_generate_counter_for_target (inLexique,  var_cas_target,  var_cas_cnt_set,  var_cas_cnt_set_code COMMA_SOURCE_FILE_AT_LINE (214)) ;
      var_cas_code.dotAssign_operation (var_cas_cnt_set_code) ;
    }
  }else{
    ::routine_retrieveTemplateString (inLexique,  var_cas_code,  GGS_string ("counter_specific") COMMA_SOURCE_FILE_AT_LINE (218)) ;
    ::routine_doReplace (inLexique,  var_cas_code,  GGS_string ("$TICK_FUNC$"),  GGS_string ("tpl_call_counter_tick") COMMA_SOURCE_FILE_AT_LINE (219)) ;
    ::routine_doReplace (inLexique,  var_cas_code,  GGS_string ("$IT_SOURCE$"),  GGS_string ("") COMMA_SOURCE_FILE_AT_LINE (220)) ;
    ::routine_generate_counter_for_target (inLexique,  var_cas_target,  var_cas_counters,  var_cas_code COMMA_SOURCE_FILE_AT_LINE (221)) ;
  }
  #ifdef DEBUG_TRACE_ENABLED
    printf ("LEAVE routine_generate_counter_specific\n") ;
  #endif
}

//---------------------------------------------------------------------------*

