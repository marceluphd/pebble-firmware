// **********************************************************************
// *                    SEGGER Microcontroller GmbH                     *
// *                        The Embedded Experts                        *
// **********************************************************************
// *                                                                    *
// *            (c) 2014 - 2020 SEGGER Microcontroller GmbH             *
// *            (c) 2001 - 2020 Rowley Associates Limited               *
// *                                                                    *
// *           www.segger.com     Support: support@segger.com           *
// *                                                                    *
// **********************************************************************
// *                                                                    *
// * All rights reserved.                                               *
// *                                                                    *
// * Redistribution and use in source and binary forms, with or         *
// * without modification, are permitted provided that the following    *
// * conditions are met:                                                *
// *                                                                    *
// * - Redistributions of source code must retain the above copyright   *
// *   notice, this list of conditions and the following disclaimer.    *
// *                                                                    *
// * - Neither the name of SEGGER Microcontroller GmbH                  *
// *   nor the names of its contributors may be used to endorse or      *
// *   promote products derived from this software without specific     *
// *   prior written permission.                                        *
// *                                                                    *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
// * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
// * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
// * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
// * DISCLAIMED.                                                        *
// * IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
// * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
// * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
// * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
// * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
// * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
// * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
// * DAMAGE.                                                            *
// *                                                                    *
// **********************************************************************

#ifndef intrinsics_h
#define intrinsics_h

#if defined(__thumb__) && (defined(__ARM_ARCH_4T__) || defined(__ARM_ARCH_5TE__) || defined(__ARM_ARCH_6__))
#define __thumb1__
#endif

#if !defined(__thumb1__)
#define __HAS_CDP__
#define __HAS_LDC__
#define __HAS_LDRT__
#define __HAS_LDRBT__
#define __HAS_MCR__
#define __HAS_MRC__
#define __HAS_MRS_MSR__
#define __HAS_STC__
#define __HAS_STRBT__
#define __HAS_STRT__
#endif

#if !defined(__ARM_ARCH_4T__)
#define __HAS_BKPT__
#endif

#if !defined(__thumb1__) && !defined(__ARM_ARCH_4T__)
#define __HAS_CDP2__
#define __HAS_CLZ__
#define __HAS_LDC2__
#define __HAS_MCR2__
#define __HAS_MRC2__
#define __HAS_MCRR__
#define __HAS_MRRC__
#define __HAS_PLD__
#define __HAS_STC2__
#endif

#if defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#define __IS_CORTEX_M__
#define __HAS_APSR__
#define __HAS_CONTROL__
#define __HAS_PRIMASK__
#elif !defined(__thumb1__)
#define __HAS_APSR__
#define __HAS_CPSR__
#endif
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#define __HAS_BASEPRI__
#define __HAS_FAULTMASK__
#endif

#if !defined(__thumb1__) && (defined(__ARM_ARCH_6__) || defined(__ARM_ARCH_6T2__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__))
#define __HAS_LDREX__
#define __HAS_MCRR2__
#define __HAS_MRRC2__
#define __HAS_STREX__
#endif

#if defined(__ARM_ARCH_6__) || defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_6T2__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__)
#define __HAS_CPS__
#define __HAS_REV__
#endif

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7A__)|| defined(__ARM_ARCH_7R__)
#define __HAS_CLREX__
#define __HAS_LDREXB__
#define __HAS_LDREXH__
#define __HAS_STREXB__
#define __HAS_STREXH__
#endif

#if defined(__ARM_ARCH_6T2__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__)
#define __HAS_LDRHT__
#define __HAS_LDRSBT__
#define __HAS_LDRSHT__
#define __HAS_RBIT__
#define __HAS_STRHT__
#endif

#if defined(__ARM_ARCH_7A__)|| defined(__ARM_ARCH_7R__)
#define __HAS_LDREXD__
#define __HAS_PLI__
#define __HAS_STREXD__
#endif

#if !defined(__thumb1__) && (defined(__ARM_ARCH_4T) || defined(__ARM_ARCH_5TE__))
#define __HAS_SWP__
#endif

#if !defined(__thumb1__) && (defined(__ARM_ARCH_6__) || defined(__ARM_ARCH_6T2__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__))
#define __HAS_SIMD__
#endif

#if !defined(__thumb1__) && (defined(__ARM_ARCH_5TE__) || defined(__ARM_ARCH_6__) || defined(__ARM_ARCH_6T2__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__))
#define __HAS_DSP__
#endif

#if defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__)
#define __HAS_BARRIER__
#define __HAS_SEV__
#define __HAS_HINTS__
#endif

#if !defined(__thumb1__) && (defined(__ARM_ARCH_VFP__) || defined(__ARM_ARCH_VFPv3_D32__) || defined(__ARM_ARCH_VFPv3_D16__))
#define __HAS_FABS__
#define __HAS_SQRT__
#endif

#if !defined(__thumb1__) && (defined(__ARM_ARCH_VFP__) || defined(__ARM_ARCH_VFPv3_D32__) || defined(__ARM_ARCH_VFPv3_D16__) || defined(__ARM_ARCH_FPV4_SP_D16__))
#define __HAS_FABSF__
#define __HAS_SQRTF__
#endif

#if defined(__HAS_SIMD__)
#define int8x4 unsigned
#define uint8x4 unsigned
#define int16x2 unsigned
#define uint16x2 unsigned
#endif

#if defined(__HAS_BKPT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief BKPT instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a BKPT instruction where \a val is a compile time constant.
*/
void __breakpoint(unsigned val);
#endif
#define __breakpoint(__imm) \
  __asm__ __volatile__("bkpt %[imm]"\
                      :\
                      : [imm] "i" (__imm))
#endif

#if defined(__HAS_CDP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief CDP instruction \ingroup Coprocessor Intrinsics \synopsis 

  \description \b \this inserts a CDP instruction. All arguments are compile time constants.
*/
void __cdp(unsigned coproc, unsigned opc1, unsigned crd, unsigned crn, unsigned crm, unsigned opc2);
#endif
#define __cdp(__coproc, __opc1, __CRd, __CRn, __CRm, __opc2) \
  __asm__ __volatile__("cdp p"#__coproc", %[opc1], c"#__CRd", c"#__CRn", c"#__CRm", %[opc2]"\
                       :\
                       : [opc1] "i" (__opc1), [opc2] "i" (__opc2))
#endif

#if defined(__HAS_CDP2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief CDP2 instruction \ingroup Coprocessor Intrinsics \synopsis 

  \description \b \this inserts a CDP2 instruction. All arguments are compile time constants.
*/
void __cdp2(unsigned coproc, unsigned opc1, unsigned crd, unsigned crn, unsigned crm, unsigned opc2);
#endif
#define __cdp2(__coproc, __opc1, __CRd, __CRn, __CRm, __opc2) \
  __asm__ __volatile__("cdp2 p"#__coproc", %[opc1], c"#__CRd", c"#__CRn", c"#__CRm", %[opc2]"\
                       :\
                       : [opc1] "i" (__opc1), [opc2] "i" (__opc2))
#endif

#if defined(__HAS_CLREX__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief CLREX instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a CLREX instruction.  
*/
void __clrex(void);
#endif
#define __clrex()\
  __asm__ __volatile__("clrex")
#endif

#if defined(__HAS_CLZ__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief CLZ instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this returns the number of leading zeros in \a val.  
*/
unsigned char __clz(unsigned val);
#endif
#define __clz(__m)\
({\
  unsigned char __d;\
  __asm__ __volatile__("clz %[d], %[m]"\
                       : [d] "=r" (__d)\
                       : [m] "r" (__m));\
  __d;\
})
#endif

#if defined(__HAS_HINTS__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief DBG instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a DBG instruction where \a option is a compile time constant.
*/
void __dbg(unsigned option);
#endif
#define __dbg(__option)\
  __asm__ __volatile__("dbg %[option]"\
                      :\
                      : [option] "i" (__option))
#endif

#if defined(__HAS_MRS_MSR__) && !defined(__IS_CORTEX_M__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Disable FIQ interrupts \ingroup Interrupt Intrinsics \synopsis

  \description \b \this sets the F bit in the CPSR and returns the previous F bit value.
*/
int __disable_fiq(void);
#endif
#if defined(__HAS_CPS__)
#define __disable_fiq()\
({\
  int __cpsr;\
  __asm__ __volatile__("mrs %[cpsr], cpsr\n"\
                       "cpsid f\n"\
                       : [cpsr] "=r"(__cpsr));\
  __cpsr & 0x40;\
})
#else
#define __disable_fiq()\
({\
  int __cpsr, __tmp;\
  __asm__ __volatile__("mrs %[cpsr], cpsr\n"\
                       "orr %[tmp], %[cpsr], #0x40\n"\
                       "msr cpsr_c, %[tmp]\n"\
                       : [cpsr] "=r"(__cpsr), [tmp] "=r"(__tmp));\
  __cpsr & 0x40;\
})
#endif
#endif

#if defined(__HAS_CPS__) || defined(__HAS_MRS_MSR__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Disable interrupt \ingroup Interrupt Intrinsics \synopsis

  \description \b \this set the PRIMASK for Cortex-M parts and sets the I and F bit in the CPSR for ARM parts.
*/
void __disable_interrupt(void);
#endif
#if defined(__IS_CORTEX_M__)
#define __disable_interrupt()\
  __asm__ __volatile__("cpsid i")
#elif defined(__HAS_CPS__)
#define __disable_interrupt()\
  __asm__ __volatile__("cpsid if")
#else
#define __disable_interrupt()\
({\
  int __cpsr, __tmp;\
  __asm__ __volatile__("mrs %[cpsr], cpsr\n"\
                       "orr %[tmp], %[cpsr], #0xC0\n"\
                       "msr cpsr_c, %[tmp]\n"\
                       : [cpsr] "=r"(__cpsr), [tmp] "=r"(__tmp));\
})
#endif
#endif

#if defined(__HAS_MRS_MSR__) && !defined(__IS_CORTEX_M__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Disable IRQ interrupts \ingroup Interrupt Intrinsics \synopsis

  \description \b \this sets the I bit in the CPSR and returns the previous I bit value.
*/
int __disable_irq(void);
#endif
#if defined(__HAS_CPS__)
#define __disable_irq()\
({\
  int __cpsr;\
  __asm__ __volatile__("mrs %[cpsr], cpsr\n"\
                       "cpsid i\n"\
                       : [cpsr] "=r"(__cpsr));\
   (__cpsr & 0x80);\
})
#else
#define __disable_irq()\
({\
  int __cpsr, __tmp;\
  __asm__ __volatile__("mrs %[cpsr], cpsr\n"\
                       "orr %[tmp], %[cpsr], #0x80\n"\
                       "msr cpsr_c, %[tmp]\n"\
                       : [cpsr] "=r"(__cpsr), [tmp] "=r"(__tmp));\
   (__cpsr & 0x80);\
})
#endif

#endif

#if defined(__HAS_BARRIER__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief DMB instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a DMB instruction.
*/
void __dmb(void);
#endif
#define __dmb()\
  __asm__ __volatile__("dmb"); 

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief DSB instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a DSB instruction.  
*/
void __dsb(void);
#endif
#define __dsb()\
  __asm__ __volatile__("dsb"); 
#endif

#if defined(__HAS_MRS_MSR__) && !defined(__IS_CORTEX_M__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Enable FIQ interrupts \ingroup Interrupt Intrinsics \synopsis

  \description \b \this clears the F bit in the CPSR.
*/
void __enable_fiq(void);
#endif
#if defined(__HAS_CPS__)
#define __enable_fiq()\
  __asm__ __volatile__("cpsie f");
#else
#define __enable_fiq()\
({\
  int __cpsr;\
  __asm__ __volatile__("mrs %[cpsr], cpsr\n"\
                       "bic %[cpsr], %[cpsr], #0x40\n"\
                       "msr cpsr_c, %[cpsr]\n"\
                       : [cpsr] "=r"(__cpsr));\
})
#endif
#endif

#if defined(__HAS_CPS__) || defined(__HAS_MRS_MSR__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Enable interrupt \ingroup Interrupt Intrinsics \synopsis

  \description \b \this clears the PRIMASK for Cortex-M parts and clears the I and F bit in the CPSR for ARM parts.
*/
void __enable_interrupt(void);
#endif
#if defined(__IS_CORTEX_M__)
#define __enable_interrupt()\
  __asm__ __volatile__("cpsie i");
#elif defined(__HAS_CPS__)
#define __enable_interrupt()\
  __asm__ __volatile__("cpsie if");
#else
#define __enable_interrupt()\
({\
  int __cpsr;\
  __asm__ __volatile__("mrs %[cpsr], cpsr\n"\
                       "bic %[cpsr], %[cpsr], #0xC0\n"\
                       "msr cpsr_c, %[cpsr]\n"\
                       : [cpsr] "=r"(__cpsr));\
})
#endif
#endif

#if defined(__HAS_MRS_MSR__)  && !defined(__IS_CORTEX_M__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Enable IRQ interrupts \ingroup Interrupt Intrinsics \synopsis

  \description \b \this clears the I bit in the CPSR.
*/
void __enable_irq(void);
#endif
#if defined(__HAS_CPS__)
#define __enable_irq()\
  __asm__ __volatile__("cpsie i");
#else
#define __enable_irq()\
({\
  int __cpsr;\
  __asm__ __volatile__("mrs %[cpsr], cpsr\n"\
                       "bic %[cpsr], %[cpsr], #0x80\n"\
                       "msr cpsr_c, %[cpsr]\n"\
                       : [cpsr] "=r"(__cpsr));\
})
#endif
#endif

#if defined(__HAS_FABS__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief VABS.F64 instruction  \ingroup VFP Intrinsics \synopsis

  \description \b \this inserts a VABS.F64 instruction. Returns the absolute value of \a val.
*/
double __fabs(double val);
#endif
#define __fabs(__m)\
({\
  double __d;\
  __asm__ __volatile__("vabs.f64 %[d], %[m]"\
                       : [d] "=w"(__d)\
                       : [m] "w"(__m));\
  __d;\
})
#endif

#if defined(__HAS_FABSF__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief VABS.F32 instruction  \ingroup VFP Intrinsics \synopsis

  \description \b \this inserts a VABS.F32 instruction. Returns the absolute value of \a val.
*/
float __fabsf(float val);
#endif
#define __fabsf(__m)\
({\
  float __d;\
  __asm__ __volatile__("vabs.f32 %[d], %[m]"\
                       : [d] "=t"(__d)\
                       : [m] "t"(__m));\
  __d;\
})
#endif

#if defined(__HAS_APSR__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Get APSR value \ingroup Register Intrinsics \synopsis

  \description \b \this returns the value of the APSR/CPSR for Cortex-M/ARM parts.  
*/
unsigned __get_APSR(void);
#endif
#if defined(__IS_CORTEX_M__)
#define __get_APSR()\
({\
  unsigned long __bp;\
  __asm__ __volatile__("mrs %[bp], APSR"\
                       : [bp] "=r"(__bp));\
  __bp;\
})
#else
#define __get_APSR()\
({\
  unsigned long __bp;\
  __asm__ __volatile__("mrs %[bp], CPSR"\
                       : [bp] "=r"(__bp));\
  __bp;\
})
#endif
#endif

#if defined(__HAS_BASEPRI__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Get BASEPRI register value \ingroup Register Intrinsics \synopsis

  \description \b \this returns the value of the Cortex-M3/M4 BASEPRI register.  
*/
unsigned __get_BASEPRI(void);
#endif
#define __get_BASEPRI()\
({\
  unsigned long __bp;\
  __asm__ __volatile__("mrs %[bp], BASEPRI"\
                       : [bp] "=r"(__bp));\
  __bp;\
})
#endif

#if defined(__HAS_CONTROL__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Get CONTROL register value \ingroup Register Intrinsics \synopsis

  \description \b \this returns the value of the Cortex-M CONTROL register.  
*/
unsigned __get_CONTROL(void);
#endif
#define __get_CONTROL()\
({\
  unsigned long __bp;\
  __asm__ __volatile__("mrs %[bp], CONTROL"\
                       : [bp] "=r"(__bp));\
  __bp;\
})
#endif

#if defined(__HAS_CPSR__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Get CPSR value \ingroup Register Intrinsics \synopsis

  \description \b \this returns the value of the ARM CPSR register.
*/
unsigned __get_CPSR(void);
#endif
#define __get_CPSR()\
({\
  unsigned long __bp;\
  __asm__ __volatile__("mrs %[bp], CPSR"\
                       : [bp] "=r"(__bp));\
  __bp;\
})
#endif

#if defined(__HAS_FAULTMASK__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Get FAULTMASK register value \ingroup Register Intrinsics \synopsis

  \description \b \this returns the value of the Cortex-M3/M4 FAULTMASK register.  
*/
unsigned __get_FAULTMASK(void);
#endif
#define __get_FAULTMASK()\
({\
  unsigned long __bp;\
  __asm__ __volatile__("mrs %[bp], FAULTMASK"\
                       : [bp] "=r"(__bp));\
  __bp;\
})
#endif

#if defined(__HAS_PRIMASK__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Get PRIMASK register value \ingroup Register Intrinsics \synopsis

  \description \b \this returns the value of the Cortex-M PRIMASK register.  
*/
unsigned __get_PRIMASK(void);
#endif
#define __get_PRIMASK()\
({\
  unsigned long __bp;\
  __asm__ __volatile__("mrs %[bp], PRIMASK"\
                       : [bp] "=r"(__bp));\
  __bp;\
})
#endif

#if defined(__HAS_BARRIER__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief ISB instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a ISB instruction.  
*/
void __isb(void);
#endif
#define __isb()\
  __asm__ __volatile__("isb")
#endif

#if defined(__HAS_LDC__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDC instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a LDC instruction where \a coproc and \a Crd are compile time constants 
  and \a ptr points to the word of data to load.
*/
void __ldc(unsigned coproc, unsigned Crd, unsigned *ptr);
#endif
#define __ldc(__coproc, __CRd, __src) \
  __asm__ __volatile__("ldc p"#__coproc", c"#__CRd", [%[src]]"\
                       :\
                       : [src] "r" (__src))

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDCL instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a LDCL instruction where \a coproc and \a Crd are compile time constants 
  and \a ptr points to the word of data to load.
*/
void __ldcl(unsigned coproc, unsigned Crd, unsigned *ptr);
#endif
#define __ldcl(__coproc, __CRd, __src) \
  __asm__ __volatile__("ldcl p"#__coproc", c"#__CRd", [%[src]]"\
                       :\
                       : [src] "r" (__src))
#endif

#if defined(__HAS_LDC2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDC2 instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a LDC2 instruction where \a coproc and \a Crd are compile time constants 
  and \a ptr points to the word of data to load.
*/
void __ldc2(unsigned coproc, unsigned Crd, unsigned *ptr);
#endif
#define __ldc2(__coproc, __CRd, __src) \
  __asm__ __volatile__("ldc2 p"#__coproc", c"#__CRd", [%[src]]"\
                       :\
                       : [src] "r" (__src))

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDC2L instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a LDC2L instruction where \a coproc and \a Crd are compile time constants 
  and \a ptr points to the word of data to load.
*/
void __ldc2l(unsigned coproc, unsigned Crd, unsigned *ptr);
#endif
#define __ldc2l(__coproc, __CRd, __src) \
  __asm__ __volatile__("ldc2l p"#__coproc", c"#__CRd", [%[src]]"\
                       :\
                       : [src] "r" (__src))
#endif

#if defined(__HAS_LDC__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDC instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a LDC instruction where \a coproc, \a Crd and \a option are compile time constants 
  and \a ptr points to the word of data to load.
*/
void __ldc_noidx(unsigned coproc, unsigned Crd, unsigned *ptr, unsigned option);
#endif
#define __ldc_noidx(__coproc, __CRd, __src, __option) \
  __asm__ __volatile__("ldc p"#__coproc", c"#__CRd", [%[src]], {%[option]}"\
                       :\
                       : [src] "r" (__src), [option] "I" (__option))

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDCL instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a LDCL instruction where \a coproc, \a Crd and \a option are compile time constants 
  and \a ptr points to the word of data to load.  
*/
void __ldcl_noidx(unsigned coproc, unsigned Crd, unsigned *ptr, unsigned option);
#endif
#define __ldcl_noidx(__coproc, __CRd, __src, __option) \
  __asm__ __volatile__("ldcl p"#__coproc", c"#__CRd", [%[src]], {%[option]}"\
                       :\
                       : [src] "r" (__src), [option] "I" (__option))
#endif

#if defined(__HAS_LDC2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDC2 instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a LDC2 instruction where \a coproc, \a Crd and \a option are compile time constants 
  and \a ptr points to the word of data to load.   
*/
void __ldc2_noidx(unsigned coproc, unsigned Crd, unsigned *ptr, unsigned option);
#endif
#define __ldc2_noidx(__coproc, __CRd, __src, __option) \
  __asm__ __volatile__("ldc2 p"#__coproc", c"#__CRd", [%[src]], {%[option]}"\
                       :\
                       : [src] "r" (__src), [option] "I" (__option))

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDC2L instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a LDC2L instruction where \a coproc, \a Crd and \a option are compile time constants 
  and \a ptr points to the word of data to load.     
*/
void __ldc2l_noidx(unsigned coproc, unsigned Crd, unsigned *ptr, unsigned option);
#endif
#define __ldc2l_noidx(__coproc, __CRd, __src, __option) \
  __asm__ __volatile__("ldc2l p"#__coproc", c"#__CRd", [%[src]], {%[option]}"\
                       :\
                       : [src] "r" (__src), [option] "I" (__option))
#endif

#if defined(__HAS_LDREX__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDREX instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDREX instruction. Returns the word of data at memory address \a ptr.
*/
unsigned __ldrex(unsigned *ptr);
#endif
#define __ldrex(__n)\
({\
  unsigned __t;\
  __asm__ __volatile__("ldrex %[t], [%[n]]"\
                       : [t] "=r"(__t)\
                       : [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_LDREXB__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDREXB instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDREXB instruction. Returns the byte of data at memory address \a ptr.
*/
unsigned __ldrexb(unsigned char *ptr);
#endif
#define __ldrexb(__n)\
({\
  unsigned __t;\
  __asm__ __volatile__("ldrexb %[t], [%[n]]"\
                       : [t] "=r"(__t)\
                       : [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_LDREXD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDREXD instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDREXD instruction. Returns the double word of data at memory address \a ptr. 
*/
unsigned long long __ldrexd(unsigned long long *ptr);
#endif
#define __ldrexd(__n)\
({\
  unsigned hi, lo;\
  __asm__ __volatile__("ldrexd %[hi], %[lo], [%[n]]"\
                       : [hi] "=r" (hi), [lo] "=r" (lo)\
                       : [n] "r" (__n));\
  ((unsigned long long)(hi)<<32)|lo;\
})
#endif

#if defined(__HAS_LDREXH__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDREXH instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDREXH instruction. Returns the half word of data at memory address \a ptr.  
*/
unsigned __ldrexh(unsigned short *ptr);
#endif
#define __ldrexh(__n)\
({\
  unsigned __t;\
  __asm__ __volatile__("ldrexh %[t], [%[n]]"\
                       : [t] "=r"(__t)\
                       : [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_LDRBT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDRBT instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDRBT instruction. Returns the byte of data at memory address \a ptr. 
*/
unsigned __ldrbt(unsigned char *ptr);
#endif
#define __ldrbt(__n)\
({\
  unsigned __t;\
  __asm__ __volatile__("ldrbt %[t], [%[n]]"\
                       : [t] "+r"(__t)\
                       : [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_LDRHT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDRHT instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDRHT instruction. Returns the half word of data at memory address \a ptr.  
*/
unsigned __ldrht(unsigned short *ptr);
#endif
#define __ldrht(__n)\
({\
  unsigned __t;\
  __asm__ __volatile__("ldrht %[t], [%[n]]"\
                       : [t] "+r"(__t)\
                       : [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_LDRT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDRT instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDRT instruction. Returns the word of data at memory address \a ptr.  
*/
unsigned __ldrt(unsigned *ptr);
#endif
#define __ldrt(__n)\
({\
  unsigned __t;\
  __asm__ __volatile__("ldrt %[t], [%[n]]"\
                       : [t] "+r"(__t)\
                       : [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_LDRSBT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDRSBT instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDRSBT instruction. Returns the sign extended byte of data at memory address \a ptr. 
*/
unsigned __ldrsbt(signed char *ptr);
#endif
#define __ldrsbt(__n)\
({\
  unsigned __t;\
  __asm__ __volatile__("ldrsbt %[t], [%[n]]"\
                       : [t] "+r"(__t)\
                       : [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_LDRSHT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief LDRSHT instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a LDRSHT instruction. Returns the sign extended half word of data at memory address \a ptr.  
*/
unsigned __ldrsht(short *ptr);
#endif
#define __ldrsht(__n)\
({\
  unsigned __t;\
  __asm__ __volatile__("ldrsht %[t], [%[n]]"\
                       : [t] "+r"(__t)\
                       : [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_MCR__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief MCR instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a MCR instruction. Where \a coproc, \a opc1, \a Crn, \a Crm and
                        \a opc2 are compile time constants and \a src is the value to write.
*/
void __mcr(unsigned coproc, unsigned opc1, unsigned src, unsigned CRn, unsigned CRm, unsigned opc2);
#endif
#define __mcr(__coproc, __opc1, __src, __CRn, __CRm, __opc2) \
  __asm__ __volatile__("mcr p"#__coproc", %[opc1], %[src], c"#__CRn", c"#__CRm", %[opc2]"\
                       :\
                       : [opc1] "i" (__opc1), [src] "r" (__src), [opc2] "i" (__opc2))
#endif

#if defined(__HAS_MCR2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief MCR2 instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a MCR2 instruction. Where \a coproc, \a opc1, \a Crn, \a Crm and
                        \a opc2 are compile time constants and \a src is the value to write.  
*/
void __mcr2(unsigned coproc, unsigned opc1, unsigned src, unsigned CRn, unsigned CRm, unsigned opc2);
#endif
#define __mcr2(__coproc, __opc1, __src, __CRn, __CRm, __opc2) \
  __asm__ __volatile__("mcr2 p"#__coproc", %[opc1], %[src], c"#__CRn", c"#__CRm", %[opc2]"\
                       :\
                       : [opc1] "i" (__opc1), [src] "r" (__src), [opc2] "i" (__opc2))
#endif

#if defined(__HAS_MCRR__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief MCRR instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a MCRR instruction. Where \a coproc, \a opc1 and \a Crn are
                        compile time constants and \a src1, \a src2 are the values to write.
*/
void __mcrr(unsigned coproc, unsigned opc1, unsigned src1, unsigned src2, unsigned CRn);
#endif
#define __mcrr(__coproc, __opc1, __src1, __src2, __CRn) \
  __asm__ __volatile__("mcrr p"#__coproc", %[opc1], %[src1], %[src2], c"#__CRn\
                       :\
                       : [opc1] "i" (__opc1), [src1] "r" (__src1), [src2] "r" (__src2))
#endif

#if defined(__HAS_MCRR2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief MCRR2 instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a MCRR2 instruction.  Where \a coproc, \a opc1 and \a Crn are
                        compile time constants and \a src1, \a src2 are the values to write.  
*/
void __mcrr2(unsigned coproc, unsigned opc1, unsigned src1, unsigned src2, unsigned CRn);
#endif
#define __mcrr2(__coproc, __opc1, __src1, __src2, __CRn) \
  __asm__ __volatile__("mcrr2 p"#__coproc", %[opc1], %[src1], %[src2], c"#__CRn\
                       :\
                       : [opc1] "i" (__opc1), [src1] "r" (__src1), [src2] "r" (__src2))
#endif

#if defined(__HAS_MRC__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief MRC instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a MRC instruction. Where \a coproc, \a opc1, \a Crn, \a Crm and
                        \a opc2 are compile time constants. \this returns the value read.
*/
unsigned __mrc(unsigned coproc, unsigned opc1, unsigned CRn, unsigned CRm, unsigned opc2);
#endif
#define __mrc(__coproc, __opc1, __CRn, __CRm, __opc2) \
({\
  unsigned long __dst;\
  __asm__ __volatile__("mrc p"#__coproc", %[opc1], %[dst], c"#__CRn", c"#__CRm", %[opc2]"\
                       : [dst] "=r" (__dst)\
                       : [opc1] "i" (__opc1), [opc2] "i" (__opc2));\
  __dst;\
})
#endif

#if defined(__HAS_MRC2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief MRC2 instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a MRC2 instruction.  Where \a coproc, \a opc1, \a Crn, \a Crm and
                        \a opc2 are compile time constants. \this returns the value read. 
*/
unsigned __mrc2(unsigned coproc, unsigned opc1, unsigned CRn, unsigned CRm, unsigned opc2);
#endif
#define __mrc2(__coproc, __opc1, __CRn, __CRm, __opc2) \
({\
  unsigned long __dst;\
  __asm__ __volatile__("mrc2 p"#__coproc", %[opc1], %[dst], c"#__CRn", c"#__CRm", %[opc2]"\
                       : [dst] "=r" (__dst)\
                       : [opc1] "i" (__opc1), [opc2] "i" (__opc2));\
  __dst;\
})
#endif

#if defined(__HAS_MRRC__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief MRRC instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a MRRC instruction. Where \a coproc, \a opc1 and \a Crn are
                        compile time constants and \a dst1, \a dst2 are the values read. 
*/
void __mrrc(unsigned coproc, unsigned opc1, unsigned *dst1, unsigned *dst2, unsigned CRn);
#endif
#define __mrrc(__coproc, __opc1, __dst1, __dst2, __CRn) \
  __asm__ __volatile__("mrrc p"#__coproc", %[opc1], %[dst1], %[dst2], c"#__CRn\
                       : [dst1] "=r" (__dst1), [dst2] "=r" (__dst2)\
                       : [opc1] "i" (__opc1))
#endif

#if defined(__HAS_MCRR2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief MRRC2 instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a MRRC2 instruction. Where \a coproc, \a opc1 and \a Crn are
                        compile time constants and \a dst1, \a dst2 are the values read.
*/
void __mrrc2(unsigned coproc, unsigned opc1, unsigned *dst1, unsigned *dst2, unsigned CRn);
#endif
#define __mrrc2(__coproc, __opc1, __dst1, __dst2, __CRn) \
  __asm__ __volatile__("mrrc2 p"#__coproc", %[opc1], %[dst1], %[dst2], c"#__CRn\
                       : [dst1] "=r" (__dst1), [dst2] "=r" (__dst2)\
                       : [opc1] "i" (__opc1))
#endif

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief NOP instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a NOP instruction.  
*/
void __nop(void);
#endif
#define __nop()\
  __asm__ __volatile__("nop");

#if defined(__HAS_PLD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief PLD instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a PLD instruction. Where \a ptr specifies the memory address.  
*/
void __pld(void *ptr);
#endif
static void __pld(void *ptr) __attribute((always_inline));
static void __pld(void *ptr)
{ 
  __asm__ __volatile__("pld %[ptr]"
                       :
                       : [ptr] "m" (ptr)); 
}
#endif
#if defined(__HAS_PLI__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief PLI instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a PLI instruction. Where \a ptr specifies the memory address.   
*/
void __pli(void *ptr);
#endif
static void __pli(void *ptr) __attribute((always_inline));
static void __pli(void *ptr)
{ 
  __asm__ __volatile__("pli %[ptr]"
                       :
                       : [ptr] "m" (ptr)); 
}
#endif

#if defined(__HAS_DSP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QADD instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QADD instruction. Returns the 32-bit saturating signed equivalent of 
                        res = val1 + val2. This operation sets the Q flag if saturation occurs.
*/
int __qadd(int val1, int val2);
#endif
#define __qadd(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qadd %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QADD8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QADD8 instruction. \b \this returns the 8-bit signed saturated equivalent of
  \item res[0] = val1[0] + val2[0]
  \item res[1] = val1[1] + val2[1]
  \item res[2] = val1[2] + val2[2]
  \item res[3] = val1[3] + val2[3] 
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
int8x4 __qadd8(int8x4 val1, int8x4 val2);
#endif
#define __qadd8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qadd8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QADD16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QADD16 instruction. \b \this returns the 16-bit signed saturated equivalent of 
  \item res[0] = val1[0] + val2[0],
  \item res[1] = val1[1] + val2[1] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
int16x2 __qadd16(int16x2 val1, int16x2 val2);
#endif
#define __qadd16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qadd16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QASX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QASX instruction. \b \this returns the 16-bit signed saturated equivalent of
  \item res[0] = val1[1] - val2[1] 
  \item res[1] = val1[1] + val2[0] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
int16x2 __qasx(int16x2 val1, int16x2 val2);
#endif
#define __qasx(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qasx %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_DSP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QDADD instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QDADD instruction. \b \this returns the 32-bit signed saturated equivalent of
                        res = val1 + (2*val2). This operation sets the Q flag if saturation occurs.
*/
int __qdadd(int val1, int val2);
#endif
#define __qdadd(__m, __n)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qdadd %[d], %[m], %[n]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QDBL instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QADD instruction. \b \this returns the 32-bit signed saturated equivalent of
                        res = val + val. This operation sets the Q flag if saturation occurs.
*/
int __qdbl(int val);
#endif
#define __qdbl(__n)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qadd %[d], %[n], %[n]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QDSUB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QDSUB instruction. \b \this returns the 32-bit signed saturated equivalent of
                        val1 - (2*val2). This operation sets the Q flag if saturation occurs.
*/
int __qdsub(int val1, int val2);
#endif
#define __qdsub(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qdsub %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Get Q flag value \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this returns the value of the Q flag.  
*/
int __qflag(void);
#endif
#define __qflag()\
  ((__get_APSR() >> 27) & 1)

#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QSAX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QSAX instruction. \b \this returns the 16-bit signed saturated equivalent of
  \item res[0] = val1[0] + val2[1] 
  \item res[1] = val1[1] - val2[0] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
int16x2 __qsax(int16x2 val1, int16x2 val2);
#endif
#define __qsax(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qsax %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_DSP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QSUB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QSUB instruction. \b \this returns the 32-bit signed saturated equivalent of 
  res=val1-val2. This operation sets the Q flag if saturation occurs.
*/
int __qsub(int val1, int val2);
#endif
#define __qsub(__m, __n)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qsub %[d], %[m], %[n]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QSUB8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QSUB8 instruction. \b \this returns the 8-bit signed saturated equivalent of
  \item res[0] = val1[0] - val2[0]
  \item res[1] = val1[1] - val2[1]
  \item res[2] = val1[2] - val2[2]
  \item res[3] = val1[3] - val2[3] 
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
int8x4 __qsub8(int8x4 val1, int8x4 val2);
#endif
#define __qsub8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qsub8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief QSUB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a QSUB16 instruction. \b \this returns the 16-bit signed saturated equivalent of
  \item res[0] = val1[0] - val2[0]
  \item res[1] = val1[1] - val2[1]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
int16x2 __qsub16(int16x2 val1, int16x2 val2);
#endif
#define __qsub16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("qsub16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_RBIT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief RBIT instruction \ingroup Reversing Intrinsics \synopsis

  \description \b \this inserts a RBIT instruction. \b \this returns the bit reversed equivalent of  \a val.
*/
unsigned __rbit(unsigned val);
#endif
#define __rbit(__n)\
({\
  unsigned long __d;\
  __asm__ __volatile__("rbit %[d], %[n]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_REV__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief REV instruction \ingroup Reversing Intrinsics \synopsis

  \description \b \this inserts a REV instruction. \b \this returns the equivalent of 
  \item res[0] = val[3]
  \item res[1] = val[2]
  \item res[2] = val[1]
  \item res[3] = val[0]
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
unsigned __rev(unsigned val);
#endif
#define __rev(__n)\
({\
  unsigned long __d;\
  __asm__ __volatile__("rev %[d], %[n]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief REV16 instruction \ingroup Reversing Intrinsics \synopsis

  \description \b \this inserts a REV16 instruction. \b \this returns the equivalent of 
  \item res[0] = val[1]
  \item res[1] = val[0]
  \item res[2] = val[3]
  \item res[3] = val[2]
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits. 
*/
unsigned __rev16(unsigned val);
#endif
#define __rev16(__n)\
({\
  unsigned long __d;\
  __asm__ __volatile__("rev16 %[d], %[n]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief REVSH instruction \ingroup Reversing Intrinsics \synopsis

  \description \b \this inserts a REVSH instruction. \b \this returns the 16-bit sign extended equivalent of
  \item res[0] = val[1]
  \item res[1] = val[0]
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits. 
*/
unsigned __revsh(unsigned val);
#endif
#define __revsh(__n)\
({\
  unsigned long __d;\
  __asm__ __volatile__("revsh %[d], %[n]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SADD8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SADD8 instruction. \b \this returns the 8-bit signed equivalent of
  \item res[0] = val1[0] + val2[0]
  \item res[1] = val1[1] + val2[1]
  \item res[2] = val1[2] + val2[2]
  \item res[3] = val1[3] + val2[3] 
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits. The GE bits of the APSR are set.
*/
int8x4 __sadd8(int8x4 val1, int8x4 val2);
#endif
#define __sadd8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("sadd8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SADD16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SADD16 instruction. \b \this returns the 16-bit signed equivalent of
  \item res[0] = val1[0] + val2[0]
  \item res[1] = val1[1] + val2[1] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. The GE bits of the APSR are set.
*/
int16x2 __sadd16(int16x2 val1, int16x2 val2);
#endif
#define __sadd16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("sadd16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SASX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SASX instruction. \b \this returns the 16-bit signed equivalent of 
  \item res[0] = val1[0] - val2[1]
  \item res[1] = val1[1] + val2[0] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. The GE bits of the APSR are set.
*/
int16x2 __sasx(int16x2 val1, int16x2 val2);
#endif
#define __sasx(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("sasx %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SEL instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SEL instruction. \b \this returns the equivalent of
  \item res[0] = GE[0] ? val1[0] : val2[0]
  \item res[1] = GE[1] ? val1[1] : val2[1]
  \item res[2] = GE[2] ? val1[2] : val2[2]
  \item res[3] = GE[3] ? val1[3] : val2[3]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
uint8x4 __sel(uint8x4 val1, uint8x4 val2);
#endif
#define __sel(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("sel %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_APSR__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Set APSR value \ingroup Register Intrinsics \synopsis

  \description \b \this sets the value of the APSR i.e. the condition bits and the GE bits.
*/
void __set_APSR(unsigned val);
#endif
#if defined(__IS_CORTEX_M__)
#define __set_APSR(__v)\
  __asm__ __volatile__("msr APSR_nzcvq, %[v]"\
                       :\
                       : [v] "r"(__v)\
                       : "cc")
#else
#define __set_APSR(__v)\
  __asm__ __volatile__("msr CPSR_fs, %[v]"\
                       :\
                       : [v] "r"(__v)\
                       : "cc")
#endif                 
#endif

#if defined(__HAS_BASEPRI__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Set BASEPRI register value \ingroup Register Intrinsics \synopsis

  \description \b \this sets the value of the Cortex-M3/M4 BASEPRI register.  
*/
void __set_BASEPRI(unsigned val);
#endif
#define __set_BASEPRI(__v)\
  __asm__ __volatile__("msr BASEPRI, %[v]"\
                       :\
                       : [v] "r"(__v))
#endif

#if defined(__HAS_CONTROL__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Set CONTROL register value \ingroup Register Intrinsics \synopsis

  \description \b \this set the value of the Cortex-M CONTROL register.  
*/
void __set_CONTROL(unsigned val);
#endif
#define __set_CONTROL(__v)\
  __asm__ __volatile__("msr CONTROL, %[v]"\
                       :\
                       : [v] "r"(__v))
#endif

#if defined(__HAS_CPSR__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Set CPSR value \ingroup Register Intrinsics \synopsis

  \description \b \this sets the value of the ARM CPSR.  
*/
void __set_CPSR(unsigned val);
#endif
#define __set_CPSR(__v)\
  __asm__ __volatile__("msr CPSR_cxfs, %[v]"\
                       :\
                       : [v] "r"(__v)\
                       : "cc")
#endif

#if defined(__HAS_FAULTMASK__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Set FAULTMASK register value \ingroup Register Intrinsics \synopsis

  \description \b \this sets the value of the Cortex-M3/M4 FAULTMASK register.  
*/
void __set_FAULTMASK(unsigned val);
#endif
#define __set_FAULTMASK(__v)\
  __asm__ __volatile__("msr FAULTMASK, %[v]"\
                       :\
                       : [v] "r"(__v))
#endif

#if defined(__IS_CORTEX_M__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief Set PRIMASK register value \ingroup Register Intrinsics \synopsis

  \description \b \this sets the value of the Cortex-M3/M4 PRIMASK register.  
*/
void __set_PRIMASK(unsigned val);
#endif
#define __set_PRIMASK(__v)\
  __asm__ __volatile__("msr PRIMASK, %[v]"\
                       :\
                       : [v] "r"(__v))
#endif

#if defined(__HAS_SEV__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SEV instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a SEV instruction.  
*/
void __sev(void);
#endif
#define __sev()\
  __asm__ __volatile__("sev"); 
#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SHADD8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SHADD8 instruction. \b \this returns the 8-bit signed equivalent of
  \item res[0] = (val1[0] + val2[0])/2
  \item res[1] = (val1[1] + val2[1])/2
  \item res[2] = (val1[2] + val2[2])/2
  \item res[3] = (val1[3] + val2[3])/2
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits. 
*/
int8x4 __shadd8(int8x4 val1, int8x4 val2);
#endif
#define __shadd8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("shadd8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SHADD16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SHADD16 instruction. \b \this returns the 16-bit signed equivalent of 
  \item res[0] = (val1[0] + val2[0])/2
  \item res[1] = (val1[1] + val2[1])/2
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
int16x2 __shadd16(int16x2 val1, int16x2 val2);
#endif
#define __shadd16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("shadd16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SHASX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SHASX instruction. \b \this returns the 16-bit signed equivalent of 
  \item res[0] = (val1[0] - val2[1])/2
  \item res[1] = (val1[1] + val2[0])/2
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
int16x2 __shasx(int16x2 val1, int16x2 val2);
#endif
#define __shasx(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("shasx %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SHSAX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SHSAX instruction. \b \this returns the 16-bit signed equivalent of 
  \item res[0] = (val1[0] + val2[1])/2
  \item res[1] = (val1[1] - val2[0])/2
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
int16x2 __shsax(int16x2 val1, int16x2 val2);
#endif
#define __shsax(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("shsax %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SHSUB8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SHSUB8 instruction.  \b \this returns the 8-bit signed equivalent of
  \item res[0] = (val1[0] - val2[0])/2
  \item res[1] = (val1[1] - val2[1])/2
  \item res[2] = (val1[2] - val2[2])/2
  \item res[3] = (val1[3] - val2[3])/2
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.  
*/
int8x4 __shsub8(int8x4 val1, int8x4 val2);
#endif
#define __shsub8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("shsub8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SHSUB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SHSUB16 instruction. \b \this returns the 16-bit signed equivalent of 
  \item res[0] = (val1[0] - val2[0])/2
  \item res[1] = (val1[1] - val2[1])/2
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
int16x2 __shsub16(int16x2 val1, int16x2 val2);
#endif
#define __shsub16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("shsub16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#endif

#if defined(__HAS_SQRT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief VSQRT.F64 instruction  \ingroup VFP Intrinsics \synopsis

  \description \b \this inserts a VSQRT.F64 instruction.  
*/
double __sqrt(double val);
#endif
#define __sqrt(__m)\
({\
  double __d;\
  __asm__ __volatile__("vsqrt.f64 %[d], %[m]"\
                       : [d] "=w"(__d)\
                       : [m] "w"(__m));\
  __d;\
})
#endif

#if defined(__HAS_SQRTF__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief VQSRT.F32 instruction  \ingroup VFP Intrinsics \synopsis

  \description \b \this inserts a VSQRT.F32 instruction.  
*/
float __sqrtf(float val);
#endif
#define __sqrtf(__m)\
({\
  float __d;\
  __asm__ __volatile__("vsqrt.f32 %[d], %[m]"\
                       : [d] "=t"(__d)\
                       : [m] "t"(__m));\
  __d;\
})
#endif

#if defined(__HAS_DSP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLABB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLABB instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[0] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smlabb(int16x2 val1, int16x2 val2, int val3);
#endif
#define __smlabb(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlabb %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLABT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLABT instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[1] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smlabt(int16x2 val1, int16x2 val2, int val3);
#endif
#define __smlabt(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlabt %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLATB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLATB instruction. \b \this returns the equivalent of 
  \item res = val1[1] * val2[0] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smlatb(int16x2 val1, int16x2 val2, int val3);
#endif
#define __smlatb(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlatb %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLATT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLATT instruction. \b \this returns the equivalent of 
  \item res = val1[1] * val2[1] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smlatt(int16x2 val1, int16x2 val2, int val3);
#endif
#define __smlatt(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlatt %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLAD instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLAD instruction. \b \this returns the 16-bit signed equivalent of 
  \item res = val1[0] * val2[0] + val1[1] * val2[1] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smlad(int16x2 val1, int16x2 val2, int val3);
#endif
#define __smlad(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlad %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLADX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLADX instruction. \b \this returns the 16-bit signed equivalent of 
  \item res = val1[0] * val2[1] + val1[1] * val2[0] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.  
*/
int __smladx(int16x2 val1, int16x2 val2, int val3);
#endif
#define __smladx(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smladx %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_DSP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLALBB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLALBB instruction.  \b \this returns the equivalent of 
  \item res = val1[0] * val2[0] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
long long __smlalbb(int16x2 val1, int16x2 val2, long long val3);
#endif
#define __smlalbb(__n, __m, __a)\
({\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("smlalbb %[lo], %[hi], %[n], %[m]"\
                       : [lo] "+r" (__lo), [hi] "+r" (__hi)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  ((long long)(__hi)<<32)|__lo;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLALBT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLALBT instruction.  \b \this returns the equivalent of 
  \item res = val1[0] * val2[1] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
long long __smlalbt(int16x2 val1, int16x2 val2, long long val3);
#endif
#define __smlalbt(__n, __m, __a)\
({\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("smlalbt %[lo], %[hi], %[n], %[m]"\
                       : [lo] "+r" (__lo), [hi] "+r" (__hi)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  ((long long)(__hi)<<32)|__lo;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLALTB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLALTB instruction.  \b \this returns the equivalent of 
  \item res = val1[1] * val2[0] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
long long __smlaltb(int16x2 val1, int16x2 val2, long long val3);
#endif
#define __smlaltb(__n, __m, __a)\
({\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("smlaltb %[lo], %[hi], %[n], %[m]"\
                       : [lo] "+r" (__lo), [hi] "+r" (__hi)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  ((long long)(__hi)<<32)|__lo;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLALTT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLALTT instruction.  \b \this returns the equivalent of 
  \item res = val1[1] * val2[1] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
long long __smlaltt(int16x2 val1, int16x2 val2, long long val3);
#endif
#define __smlaltt(__n, __m, __a)\
({\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("smlaltt %[lo], %[hi], %[n], %[m]"\
                       : [lo] "+r" (__lo), [hi] "+r" (__hi)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  ((long long)(__hi)<<32)|__lo;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLAWB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLAWB instruction. \b \this returns the equivalent of 
  \item res = (val1 * val2[0] + (val3 &lt;&lt; 16)) &gt;&gt; 16
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smlawb(int val1, int16x2 val2, int val3);
#endif
#define __smlawb(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlawb %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLAWT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLAWT instruction. \b \this returns the equivalent of 
  \item res = (val1 * val2[1] + (val3 &lt;&lt; 16)) &gt;&gt; 16
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smlawt(int val1, int16x2 val2, int val3);
#endif
#define __smlawt(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlawt %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLALD instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLALD instruction.  \b \this returns the equivalent of 
  \item res = val1[0] * val2[0] + val1[1] * val2[1] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
long long __smlald(int16x2 val1, int16x2 val2, long long val3);
#endif
#define __smlald(__n, __m, __a)\
({\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("smlald %[lo], %[hi], %[n], %[m]"\
                       : [lo] "+r" (__lo), [hi] "+r" (__hi)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  ((long long)(__hi)<<32)|__lo;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLALDX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLALDX instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[1] + val1[1] * val2[0] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.  
*/
long long __smlaldx(int16x2 val1, int16x2 val2, long long val3);
#endif
#define __smlaldx(__n, __m, __a)\
({\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("smlaldx %[lo], %[hi], %[n], %[m]"\
                       : [lo] "+r" (__lo), [hi] "+r" (__hi)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  ((long long)(__hi)<<32)|__lo;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLSD instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLSD instruction.  \b \this returns the equivalent of 
  \item res = val1[0] * val2[0] - val1[1] * val2[1] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smlsd(int16x2 val1, int16x2 val2, int val3);
#endif
#define __smlsd(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlsd %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLSDX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLSDX instruction.  \b \this returns the equivalent of 
  \item res = val1[0] * val2[1] - val1[1] * val2[0] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition. 
*/
int __smlsdx(int16x2 val1, int16x2 val2, int val3);
#endif
#define __smlsdx(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smlsdx %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLSLD instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLSLD instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[0] - val1[1] * val2[1] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.    
*/
long long __smlsld(int16x2 val1, int16x2 val2, long long val3);
#endif
#define __smlsld(__n, __m, __a)\
({\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("smlsld %[lo], %[hi], %[n], %[m]"\
                       : [lo] "+r" (__lo), [hi] "+r" (__hi)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  ((long long)(__hi)<<32)|__lo;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMLSLDX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMLSLDX instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[1] - val1[1] * val2[0] + val3
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
long long __smlsldx(int16x2 val1, int16x2 val2, long long val3);
#endif
#define __smlsldx(__n, __m, __a)\
({\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("smlsldx %[lo], %[hi], %[n], %[m]"\
                       : [lo] "+r" (__lo), [hi] "+r" (__hi)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  ((long long)(__hi)<<32)|__lo;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMUAD instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMUAD instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[0] + val1[1] * val2[1]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.
*/
int __smuad(int16x2 val1, int16x2 val2);
#endif
#define __smuad(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smuad %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMUADX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMUADX instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[1] + val1[1] * val2[0]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the Q flag if overflow occurs on the addition.  
*/
int __smuadx(int16x2 val1, int16x2 val2);
#endif
#define __smuadx(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smuadx %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_DSP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMULBB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMULBB instruction.  \b \this returns the equivalent of 
  \item res = val1[0] * val2[0]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
int __smulbb(int16x2 val1, int16x2 val2);
#endif
#define __smulbb(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smulbb %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
   __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMULBT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMULBT instruction.  \b \this returns the equivalent of 
  \item res = val1[0] * val2[1]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
int __smulbt(int16x2 val1, int16x2 val2);
#endif
#define __smulbt(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smulbt %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
   __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMULTB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMULTB instruction.  \b \this returns the equivalent of 
  \item res = val1[1] * val2[0]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
int __smultb(int16x2 val1, int16x2 val2);
#endif
#define __smultb(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smultb %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
   __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMULTT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMULTT instruction.  \b \this returns the equivalent of 
  \item res = val1[1] * val2[1]
  \n
  \n where [1] is the lower 16 bits and [1] is the upper 16 bits. 
*/
int __smultt(int16x2 val1, int16x2 val2);
#endif
#define __smultt(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smultt %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
   __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMULWB instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMULWB instruction. \b \this returns the equivalent of 
  \item res = (val1[0] * val2) &gt;&gt; 16
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.   
*/
int __smulwb(int16x2 val1, int val2);
#endif
#define __smulwb(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smulwb %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMULWT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMULWT instruction. \b \this returns the equivalent of 
  \item res = (val1[1] * val2) &gt;&gt; 16
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.   
*/
int __smulwt(int16x2 val1, int val2);
#endif
#define __smulwt(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smulwt %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMUSD instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMUSD instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[0] - val1[1] * val2[1]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.  
*/
int __smusd(int16x2 val1, int16x2 val2);
#endif
#define __smusd(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smusd %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SMUSDX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SMUSDX instruction. \b \this returns the equivalent of 
  \item res = val1[0] * val2[1] - val1[1] * val2[0]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.  
*/
int __smusdx(int16x2 val1, int16x2 val2);
#endif
#define __smusdx(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("smusdx %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SSAT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SSAT instruction. \b \this returns \a val saturated to the signed range of \a sat where \a sat is a compile time constant.
*/
int __ssat(int val, unsigned sat);
#endif
#define __ssat(__n, __imm)\
({\
  unsigned long __d;\
  __asm__ __volatile__("ssat %[d], %[imm], %[n]"\
                       : [d] "=r" (__d)\
                       : [imm] "i" (__imm), [n] "r" (__n)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SSAT16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SSAT16 instruction. \b \this returns the equivalent of 
  \item res[0] = val[0] saturated to the signed range of sat
  \item res[1] = val[1] saturated to the signed range of sat
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits and \a sat is a compile time constant.  
*/
int16x2 __ssat16(int16x2 val, unsigned sat);
#endif
#define __ssat16(__n, __imm)\
({\
  unsigned long __d;\
  __asm__ __volatile__("ssat16 %[d], %[imm], %[n]"\
                       : [d] "=r" (__d)\
                       : [imm] "i" (__imm), [n] "r" (__n)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SSAX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SSAX instruction. \b \this returns the equivalent of 
  \item res[0] = val1[0] + val2[1]
  \item res[1] = val1[1] - val2[0]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the GE bits.
*/
int16x2 __ssax(int16x2 val1, int16x2 val2);
#endif
#define __ssax(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("ssax %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SSUB8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SSUB8 instruction. \b \this returns the 8-bit signed equivalent of
  \item res[0] = val1[0] - val2[0]
  \item res[1] = val1[1] - val2[1]
  \item res[2] = val1[2] - val2[2]
  \item res[3] = val1[3] - val2[3] 
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits. The GE bits of the APSR are set.
*/
int8x4 __ssub8(int8x4 val1, int8x4 val2);
#endif
#define __ssub8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("ssub8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SSUB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SSUB16 instruction. \b \this returns the 16-bit signed equivalent of
  \item res[0] = val1[0] - val2[0]
  \item res[1] = val1[1] - val2[1] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. The GE bits of the APSR are set.
*/
int16x2 __ssub16(int16x2 val1, int16x2 val2);
#endif
#define __ssub16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("ssub16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_STC__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STC instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a STC instruction where \a coproc and \a Crd are compile time constants 
  and \a ptr points to the word of data to store.
*/
void __stc(unsigned coproc, unsigned Crd, unsigned *ptr);
#endif
#define __stc(__coproc, __CRd, __src) \
  __asm__ __volatile__("stc p"#__coproc", c"#__CRd", [%[src]]"\
                       :\
                       : [src] "r" (__src))

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STCL instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a STCL instruction where \a coproc and \a Crd are compile time constants 
  and \a ptr points to the word of data to store.
*/
void __stcl(unsigned coproc, unsigned Crd, unsigned *ptr);
#endif
#define __stcl(__coproc, __CRd, __src) \
  __asm__ __volatile__("stcl p"#__coproc", c"#__CRd", [%[src]]"\
                       :\
                       : [src] "r" (__src))
#endif

#if defined(__HAS_STC2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STC2 instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a STC2 instruction where \a coproc and \a Crd are compile time constants 
  and \a ptr points to the word of data to store.
*/
void __stc2(unsigned coproc, unsigned Crd, unsigned *ptr);
#endif
#define __stc2(__coproc, __CRd, __src) \
  __asm__ __volatile__("stc2 p"#__coproc", c"#__CRd", [%[src]]"\
                       :\
                       : [src] "r" (__src))

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STC2L instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a STC2L instruction where \a coproc and \a Crd are compile time constants 
  and \a ptr points to the word of data to store.
*/
void __stc2l(unsigned coproc, unsigned Crd, unsigned *ptr);
#endif
#define __stc2l(__coproc, __CRd, __src) \
  __asm__ __volatile__("stc2l p"#__coproc", c"#__CRd", [%[src]]"\
                       :\
                       : [src] "r" (__src))
#endif

#if defined(__HAS_STC__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STC instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a STC instruction where \a coproc, \a Crd and \a option are compile time constants 
  and \a ptr points to the word of data to store.
*/
void __stc_noidx(unsigned coproc, unsigned Crd, unsigned *ptr, unsigned option);
#endif
#define __stc_noidx(__coproc, __CRd, __src, __option) \
  __asm__ __volatile__("stc p"#__coproc", c"#__CRd", [%[src]], {%[option]}"\
                       :\
                       : [src] "r" (__src), [option] "I" (__option))

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STCL instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a STCL instruction where \a coproc, \a Crd and \a option are compile time constants 
  and \a ptr points to the word of data to store.
*/
void __stc_noidx(unsigned coproc, unsigned Crd, unsigned *ptr, unsigned option);
#endif
#define __stcl_noidx(__coproc, __CRd, __src, __option) \
  __asm__ __volatile__("stcl p"#__coproc", c"#__CRd", [%[src]], {%[option]}"\
                       :\
                       : [src] "r" (__src), [option] "I" (__option))
#endif

#if defined(__HAS_STC2__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STC instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a STC instruction where \a coproc, \a Crd and \a option are compile time constants 
  and \a ptr points to the word of data to store.
*/
void __stc_noidx(unsigned coproc, unsigned Crd, unsigned *ptr, unsigned option);
#endif
#define __stc2_noidx(__coproc, __CRd, __src, __option) \
  __asm__ __volatile__("stc2 p"#__coproc", c"#__CRd", [%[src]], {%[option]}"\
                       :\
                       : [src] "r" (__src), [option] "I" (__option))

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STC2L instruction \ingroup Coprocessor Intrinsics \synopsis

  \description \b \this inserts a STC2L instruction where \a coproc, \a Crd and \a option are compile time constants 
  and \a ptr points to the word of data to store.
*/
void __stc_noidx(unsigned coproc, unsigned Crd, unsigned *ptr, unsigned option);
#endif
#define __stc2l_noidx(__coproc, __CRd, __src, __option) \
  __asm__ __volatile__("stc2l p"#__coproc", c"#__CRd", [%[src]], {%[option]}"\
                       :\
                       : [src] "r" (__src), [option] "I" (__option))
#endif

#if defined(__HAS_STREX__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STREX instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a STREX instruction.  
*/
int __strex(unsigned val, unsigned *ptr);
#endif
#define __strex(__t, __n)\
({\
  int __d;\
  __asm__ __volatile__("strex %[d], %[t], [%[n]]"\
                       : [d] "+r"(__d)\
                       : [t] "r" (__t), [n] "r" (__n));\
  __d;\
})
#endif

#if defined(__HAS_STREXB__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STREXB instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a STREXB instruction.  
*/
int __strexb(unsigned char val, unsigned *char ptr);
#endif
#define __strexb(__t, __n)\
({\
  int __d;\
  __asm__ __volatile__("strexb %[d], %[t], [%[n]]"\
                       : [d] "+r"(__d)\
                       : [t] "r" (__t), [n] "r" (__n));\
  __d;\
})
#endif

#if defined(__HAS_STREXD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STREXD instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a STREXD instruction.  
*/
int __strexd(unsigned long long val, unsigned *long long ptr);
#endif
#define __strexd(__a, __n)\
({\
  int __d;\
  unsigned __hi = (__a >> 32);\
  unsigned __lo = __a;\
  __asm__ __volatile__("strexd %[d], %[hi], %[lo], [%[n]]"\
                       : [d] "+r"(__d)\
                       : [hi] "r" (__hi), [lo] "r" (__lo), [n] "r" (__n));\
  __d;\
})
#endif

#if defined(__HAS_STREXH__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STREXH instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a STREXH instruction.  
*/
int __strexh(unsigned short val, unsigned *short ptr);
#endif
#define __strexh(__t, __n)\
({\
  int __d;\
  __asm__ __volatile__("strexh %[d], %[t], [%[n]]"\
                       : [d] "+r"(__d)\
                       : [t] "r" (__t), [n] "r" (__n));\
  __d;\
})
#endif

#if defined(__HAS_STRBT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STRBT instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a STRBT instruction.  
*/
void __strbt(unsigned char val, unsigned char *ptr);
#endif
#define __strbt(__t, __n)\
  __asm__ __volatile__("strbt %[t], [%[n]]"\
                       : \
                       : [t] "r" (__t), [n] "r" (__n))
#endif

#if defined(__HAS_STRHT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STRHT instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a STRHT instruction.  
*/
void __strht(unsigned short val, unsigned short *ptr);
#endif
#define __strht(__t, __n)\
  __asm__ __volatile__("strht %[t], [%[n]]"\
                       :\
                       : [t] "r" (__t), [n] "r" (__n))
#endif

#if defined(__HAS_STRT__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief STRT instruction \ingroup Load/Store Intrinsics \synopsis

  \description \b \this inserts a STRT instruction.  
*/
void __strt(unsigned val, unsigned *ptr);
#endif
#define __strt(__t, __n)\
  __asm__ __volatile__("strt %[t], [%[n]]"\
                       :\
                       : [t] "r" (__t), [n] "r" (__n));
#endif

#if defined(__HAS_SWP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SWP instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a SWP instruction.  
*/
unsigned __swp(unsigned val, unsigned *ptr);
#endif
#define __swp(__t2, __n)\
({\
  unsigned __t;\
  __asm__ __volatile__("swp %[t], %[t2], [%[n]]"\
                       : [t] "+r" (__t)\
                       : [t2] "r" (__t2), [n] "r" (__n));\
  __t;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SWPB instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a SWPB instruction.  
*/
unsigned __swpb(unsigned char val, unsigned char *ptr);
#endif
#define __swpb(__t2, __n)\
({\
  unsigned __t;\
  __asm__ __volatile__("swpb %[t], %[t2], [%[n]]"\
                       : [t] "+r" (__t)\
                       : [t2] "r" (__t2), [n] "r" (__n));\
  __t;\
})
#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SXTAB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SXTAB16 instruction. \b \this returns the 16-bit signed equivalent of
  \item res[0] = val1[0] + (short)val2[0]
  \item res[1] = val1[1] + (short)val2[2]
  \n
  \n where res[0] &amp; val1[0] are the lower 16 bits, res[1] &amp; val1[1] are the upper 
  16 bits, val2[0] is the lower 8 bits and val2[2] is the 8 bits starting at bit position 16.
*/
int16x2 __sxtab16(int16x2 val1, uint8x4 val2);
#endif
#define __sxtab16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("sxtab16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief SXTB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a SXTB16 instruction.  \b \this returns the 16-bit signed equivalent of
  \item res[0] = (short)val[0]
  \item res[1] = (short)val[2]
  \n
  \n where res[0] is the lower 16 bits, res[1] is the upper 16 bits, val[0] is the lower 8 bits and val[2] is the 8 bits 
  starting at bit position 16.
*/
int16x2 __sxtb16(int8x4 val);
#endif
#define __sxtb16(__m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("sxtb16 %[d], %[m]"\
                       : [d] "=r" (__d)\
                       : [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UADD8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UADD8 instruction. \b \this returns the 8-bit unsigned equivalent of
  \item res[0] = val1[0] + val2[0]
  \item res[1] = val1[1] + val2[1]
  \item res[2] = val1[2] + val2[2]
  \item res[3] = val1[3] + val2[3] 
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits. The GE bits of the APSR are set. 
*/
uint8x4 __uadd8(uint8x4 val1, uint8x4 val2);
#endif
#define __uadd8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uadd8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UADD16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UADD16 instruction. \b \this returns the 16-bit unsigned equivalent of
  \item res[0] = val1[0] + val2[0]
  \item res[1] = val1[1] + val2[1] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. The GE bits of the APSR are set.
*/
uint16x2 __uadd16(uint16x2 val1, uint16x2 val2);
#endif
#define __uadd16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uadd16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UASX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UASX instruction. \b \this returns the 16-bit unsigned equivalent of 
  \item res[0] = val1[0] - val2[1]
  \item res[1] = val1[1] + val2[0] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. The GE bits of the APSR are set.
*/
uint16x2 __uasx(uint16x2 val1, uint16x2 val2);
#endif
#define __uasx(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uasx %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UHADD8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UHADD8 instruction. \b \this returns the 8-bit unsigned equivalent of
  \item res[0] = (val1[0] + val2[0])/2
  \item res[1] = (val1[1] + val2[1])/2
  \item res[2] = (val1[2] + val2[2])/2
  \item res[3] = (val1[3] + val2[3])/2
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.  
*/
uint8x4 __uhadd8(uint8x4 val1, uint8x4 val2);
#endif
#define __uhadd8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uhadd8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UHADD16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UHADD16 instruction.   \b \this returns the 16-bit unsigned equivalent of 
  \item res[0] = (val1[0] + val2[0])/2
  \item res[1] = (val1[1] + val2[1])/2
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
uint16x2 __uhadd16(uint16x2 val1, uint16x2 val2);
#endif
#define __uhadd16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uhadd16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UHASX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis
  
  \description \b \this inserts a UHASX instruction. \b \this returns the 16-bit unsigned equivalent of 
  \item res[0] = (val1[0] - val2[1])/2
  \item res[1] = (val1[1] + val2[0])/2
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
uint16x2 __uhasx(uint16x2 val1, uint16x2 val2);
#endif
#define __uhasx(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uhasx %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UHSAX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UHSAX instruction. \b \this returns the 16-bit unsigned equivalent of 
  \item res[0] = (val1[0] + val2[1])/2
  \item res[1] = (val1[1] - val2[0])/2
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
uint16x2 __uhsax(uint16x2 val1, uint16x2 val2);
#endif
#define __uhsax(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uhsax %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UHSUB8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UHSUB8 instruction. \b \this returns the 8-bit unsigned equivalent of
  \item res[0] = (val1[0] - val2[0])/2
  \item res[1] = (val1[1] - val2[1])/2
  \item res[2] = (val1[2] - val2[2])/2
  \item res[3] = (val1[3] - val2[3])/2
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.  
*/
uint8x4 __uhsub8(uint8x4 val1, uint8x4 val2);
#endif
#define __uhsub8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uhsub8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UHSUB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UHSUB16 instruction. \b \this returns the 16-bit unsigned equivalent of 
  \item res[0] = (val1[0] - val2[0])/2
  \item res[1] = (val1[1] - val2[1])/2
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
uint16x2 __uhsub16(uint16x2 val1, uint16x2 val2);
#endif
#define __uhsub16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uhsub16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UQADD8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UQADD8 instruction. \b \this returns the 8-bit unsigned saturated equivalent of
  \item res[0] = val1[0] + val2[0]
  \item res[1] = val1[1] + val2[1]
  \item res[2] = val1[2] + val2[2]
  \item res[3] = val1[3] + val2[3] 
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
uint8x4 __uqadd8(uint8x4 val1, uint8x4 val2);
#endif
#define __uqadd8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uqadd8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UQADD16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UQADD16 instruction. \b \this returns the 16-bit unsigned saturated equivalent of
  \item res[0] = val1[0] + val2[0]
  \item res[1] = val1[1] + val2[1] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
uint16x2 __uqadd16(uint16x2 val1, uint16x2 val2);
#endif
#define __uqadd16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uqadd16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UQASX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UQASX instruction. \b \this returns the 16-bit signed saturated equivalent of 
  \item res[0] = val1[0] - val2[1]
  \item res[1] = val1[1] + val2[0] 
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits.
*/
uint16x2 __uqasx(uint16x2 val1, uint16x2 val2);
#endif
#define __uqasx(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uqasx %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UQSAX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UQSAX instruction.  \b \this returns the 16-bit signed saturated equivalent of 
  \item res[0] = val1[0] + val2[1]
  \item res[1] = val1[1] - val2[0]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
*/
uint16x2 __uqsax(uint16x2 val1, uint16x2 val2);
#endif
#define __uqsax(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uqsax %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UQSUB8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UQSUB8 instruction. \b \this returns the 8-bit unsigned saturated equivalent of
  \item res[0] = val1[0] - val2[0]
  \item res[1] = val1[1] - val2[1]
  \item res[2] = val1[2] - val2[2]
  \item res[3] = val1[3] - val2[3] 
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
uint8x4 __uqsub8(uint8x4 val1, uint8x4 val2);
#endif
#define __uqsub8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uqsub8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UQSUB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UQSUB16 instruction.  \b \this returns the 16-bit unsigned saturated equivalent of
  \item res[0] = val1[0] - val2[0]
  \item res[1] = val1[1] - val2[1]
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
uint16x2 __uqsub16(uint16x2 val1, uint16x2 val2);
#endif
#define __uqsub16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uqsub16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief USAD8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a USAD8 instruction. \b \this returns the 8-bit unsigned equivalent of
  \item res = abs(val1[0] - val2[0]) + abs(val1[1] - val2[1]) + (val1[2] - val2[2]) + (val1[3] - val2[3])
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
unsigned __usad8(uint8x4 val1, uint8x4 val2);
#endif
#define __usad8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("usad8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief USADA8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a USADA8 instruction. \b \this returns the 8-bit unsigned equivalent of
  \item res = abs(val1[0] - val2[0]) + abs(val1[1] - val2[1]) + (val1[2] - val2[2]) + (val1[3] - val2[3]) + val3
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
unsigned __usad8a(uint8x4 val1, uint8x4 val2, unsigned val3);
#endif
#define __usada8(__n, __m, __a)\
({\
  unsigned long __d;\
  __asm__ __volatile__("usada8 %[d], %[n], %[m], %[a]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m), [a] "r" (__a)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_DSP__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief USAT instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a USAT instruction. \b \this returns \a val saturated to the unsigned range of \a sat where \a sat is a compile time constant.
*/
int __usat(int val, unsigned sat);
#endif
#define __usat(__n, __imm)\
({\
  unsigned long __d;\
  __asm__ __volatile__("usat %[d], %[imm], %[n]"\
                       : [d] "=r" (__d)\
                       : [imm] "i" (__imm), [n] "r" (__n)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_SIMD__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief USAT16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a USAT16 instruction.  \b \this returns the equivalent of 
  \item res[0] = val[0] saturated to the unsigned range of sat
  \item res[1] = val[1] saturated to the unsigned range of sat
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits and \a sat is a compile time constant.  
*/
int16x2 __usat16(int16x2 val, const unsigned sat);
#endif
#define __usat16(__n, __imm)\
({\
  unsigned long __d;\
  __asm__ __volatile__("usat16 %[d], %[imm], %[n]"\
                       : [d] "=r" (__d)\
                       : [imm] "i" (__imm), [n] "r" (__n)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief USAX instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a USAX instruction. \b \this returns the equivalent of 
  \item res[0] = val1[0] + val2[1]
  \item res[1] = val1[1] - val2[0]
  \n
  \n where [0] is the lower 16 bits and [1] is the upper 16 bits. 
  This operation sets the GE bits.
*/
int16x2 __usax(int16x2 val1, int16x2 val2);
#endif
#define __usax(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("usax %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief USUB8 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a USUB8 instruction. \b \this returns the 8-bit unsigned equivalent of
  \item res[0] = val1[0] - val2[0]
  \item res[1] = val1[1] - val2[1]
  \item res[2] = val1[2] - val2[2]
  \item res[3] = val1[3] - val2[3] 
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
uint8x4 __usub8(uint8x4 val1, uint8x4 val2);
#endif
#define __usub8(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("usub8 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief USUB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a USUB16 instruction.  \b \this returns the 16-bit unsigned equivalent of
  \item res[0] = val1[0] - val2[0]
  \item res[1] = val1[1] - val2[1]
  \n
  \n where [0] is the lower 8 bits and [3] is the upper 8 bits.
*/
uint16x2 __uqsub16(uint16x2 val1, uint16x2 val2);
#endif
#define __usub16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("usub16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UXTAB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UXTAB16 instruction.  \b \this returns the 16-bit unsigned equivalent of
  \item res[0] = val1[0] + (unsigned short)val2[0]
  \item res[1] = val1[1] + (unsigned short)val2[2]
  \n
  \n where res[0] &amp; val1[0] are the lower 16 bits, res[1] &amp; val1[1] are the upper 
  16 bits, val2[0] is the lower 8 bits and val2[2] is the 8 bits starting at bit position 16.
*/
int16x2 __uxtab16(int16x2 val1, uint8x4 val2);
#endif
#define __uxtab16(__n, __m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uxtab16 %[d], %[n], %[m]"\
                       : [d] "=r" (__d)\
                       : [n] "r" (__n), [m] "r" (__m)\
                       : "cc");\
  __d;\
})

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief UXTB16 instruction \ingroup DSP &amp; SIMD Intrinsics \synopsis

  \description \b \this inserts a UXTB16 instruction. \b \this returns the 16-bit unsigned equivalent of
  \item res[0] = (unsigned short)val[0]
  \item res[1] = (unsigned short)val[2]
  \n
  \n where res[0] is the lower 16 bits, res[1] is the upper 16 bits, val[0] is the lower 8 bits and val[2] is the 8 bits 
  starting at bit position 16.
*/
int16x2 __uxtb16(int8x4 val);
#endif
#define __uxtb16(__m)\
({\
  unsigned long __d;\
  __asm__ __volatile__("uxtb16 %[d], %[m]"\
                       : [d] "=r" (__d)\
                       : [m] "r" (__m)\
                       : "cc");\
  __d;\
})
#endif

#if defined(__HAS_HINTS__)
#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief WFE instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a WFE instruction.  
*/
void __wfe(void);
#endif
#define __wfe()\
  __asm__ __volatile__("wfe")

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief WFI instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a WFI instruction.  
*/
void __wfi(void);
#endif
#define __wfi()\
  __asm__ __volatile__("wfi")

#ifdef __CROSSWORKS_DOCUMENTATION
/*! \brief YIELD instruction \ingroup Misc Intrinsics \synopsis

  \description \b \this inserts a YIELD instruction.  
*/
void __yield(void);
#endif
#define __yield()\
  __asm__ __volatile__("yield")
#endif
#endif
