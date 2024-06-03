/*
 * dp83869.h
 *
 *  Created on: 29-May-2024
 *      Author: AARTECH_Gourav_Sankadia
 */

#ifndef DP83869_H_
#define DP83869_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/******************************************************************************/
/******************************************************************************/
/*! \brief Check if PHY address is valid (0 - 31). */
#define ENETPHY_IS_ADDR_VALID(addr) ((addr) <= 31U)

/*! \brief Macro to set bit at given bit position. */
#define ENETPHY_BIT(n) (1U << (n))

/*! \brief Macro to check if bit at given bit position is set. */
#define ENETPHY_IS_BIT_SET(val, n) (((val)&ENETPHY_BIT(n)) != 0U)

/*! \brief Macro to get the size of an array. */
#define ENETPHY_ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

/******************************************************************************/
/******************************************************************************/

//PHY Identification register form a unique identification register for dp83869
/*! \brief Basic Mode Control Register (BMCR) */
#define PHY_BMCR (0x00U)

/*! \brief Basic Mode Status Register (BMSR) */
#define PHY_BMSR (0x01U)

/*! \brief PHY Identifier Register #1 (PHYIDR1) */
#define PHY_PHYIDR1 (0x02U)

/*! \brief PHY Identifier Register #2 (PHYIDR2) */
#define PHY_PHYIDR2 (0x03U)

/*! \brief Auto-Negotiation Advertisement Register (ANAR) */
#define PHY_ANAR (0x04U)

/*! \brief Auto-Negotiation Link Partner Abilitiy Register (ANLPAR) */
#define PHY_ANLPAR (0x05U)

/*! \brief Auto-Negotiation Expansion Register (ANER) */
#define PHY_ANER

/*! \brief Auto-Negotiation NP TX Register (ANNPTR) */
#define PHY_ANNPTR (0x07U)

/*! \brief Auto-Neg NP RX Register (ANNPRR) */
#define PHY_ANNPRR (0x08U)

/*! \brief 1000BASE-T Control Register (GIGCR) */
#define PHY_GIGCR (0x09U)

/*! \brief 1000BASE-T Status Register (GIGSR) */
#define PHY_GIGSR (0x0AU)

/*! \brief MMD ACCESS  Control Register */
#define PHY_MMD_CR (0x0DU)

/*! \brief MMD Data Register */
#define PHY_MMD_DR (0x0EU)

/*! \brief 1000BASE-T Extended Status Register (GIGESR) */
#define PHY_GIGESR (0x0EF)

/*! \brief PHY Status Register (phystatus) */
#define PHY_STATUS (0x11)

/******************************************************************************/
/******************************************************************************/

/*===================================================================*/
/*                        register macros                                  */
/*====================================================================*/

/*********************BASIC MODE CONTROL REGISTER**********************/
#define PHY_ID_SHIFT (16U)
/* BMCR register definitions */
#define BMCR_RESET     ENETPHY_BIT(15)
#define BMCR_LOOPBACK  ENETPHY_BIT(14)
#define BMCR_SPEED100  ENETPHY_BIT(13)
#define BMCR_ANEN      ENETPHY_BIT(12)
#define BMCR_PWRDOWN   ENETPHY_BIT(11)
#define BMCR_ISOLATE   ENETPHY_BIT(10)
#define BMCR_ANRESTART ENETPHY_BIT(9)
#define BMCR_FD        ENETPHY_BIT(8)
#define BMCR_SPEED1000 ENETPHY_BIT(6)
/*********************BASIC MODE STATUS REGISTER**********************/

#define BMSR_100FD     ENETPHY_BIT(14)
#define BMSR_100HD     ENETPHY_BIT(13)
#define BMSR_10FD      ENETPHY_BIT(12)
#define BMSR_10HD      ENETPHY_BIT(11)
#define BMSR_GIGEXTSTS ENETPHY_BIT(8)
#define BMSR_ANSTATUS  ENETPHY_BIT(5)
#define BMSR_ANCAPABLE ENETPHY_BIT(3)
#define BMSR_LINKSTS   ENETPHY_BIT(2)
//#define BMSR_EXTCAP       ENETPHY_BIT(0)
#define BMSR_ANINCOMPLETE (0x00)

/********************* PHYIDR1 and PHYIDR2 REGISTER**********************/
#define PHYIDR1_OUI_OFFSET (6U)
/*************************************************************************/

#define PHYIDR2_OUI_MASK    (0xFC00U)
#define PHYIDR2_VMDL_MASK   (0x03F0U)
#define PHYIDR2_VREV_MASK   (0x000FU)
#define PHYIDR2_OUI_OFFSET  (10U)
#define PHYIDR2_VMDL_OFFSET (4U)
#define PHYIDR2_VREV_OFFSET (0U)

/*********************AUTONEGOTIATION ADVERTISEMENT REGISTER**********************/
#define ANAR_100FD ENETPHY_BIT(8)
#define ANAR_100HD ENETPHY_BIT(7)
#define ANAR_10FD  ENETPHY_BIT(6)
#define ANAR_10HD  ENETPHY_BIT(5)
#define ANAR_802P3 ENETPHY_BIT(0)
#define ANAR_100   (ANAR_100FD | ANAR_100HD)
#define ANAR_10    (ANAR_10FD | ANAR_10HD)

/*********************AUTONEGOTIATION LINK PARTNER ABILITY REGISTER************/

#define ANLPAR_100FD ENETPHY_BIT(8)
#define ANLPAR_100HD ENETPHY_BIT(7)
#define ANLPAR_10FD  ENETPHY_BIT(6)
#define ANLPAR_10HD  ENETPHY_BIT(5)
#define ANLPAR_100   (ANLPAR_100FD | ANLPAR_100HD)
#define ANLPAR_10    (ANLPAR_10FD | ANLPAR_10HD)

/*********************AUTONEGOTIATION EXPANSION REGISTER************/
/*********************AUTONEGOTIATION NEXT PAGE TRANSMIT REGISTER************/
/*********************AUTONEGOTIATION NEXT PAGE RECIEVE REGISTER************/
/*********************GENRAL CONFIGURATION 1 REGISTER********************/

#define GIGCR_MASTERCFG ENETPHY_BIT(12)
#define GIGCR_MASTEREN  ENETPHY_BIT(11)
#define GIGCR_1000FD    ENETPHY_BIT(9)
#define GIGCR_1000HD    ENETPHY_BIT(8)
#define GIGCR_1000      (GIGCR_1000FD | GIGCR_1000HD)

/******************* GENRAL STATUS REGISTER **********************/
#define GIGSR_MASTERRES ENETPHY_BIT(14)
#define GIGSR_LOCALSTS  ENETPHY_BIT(13)
#define GIGSR_LPARSTS   ENETPHY_BIT(12)
#define GIGSR_1000FD    ENETPHY_BIT(11)
#define GIGSR_1000HD    ENETPHY_BIT(10)
#define GIGSR_1000      (GIGSR_1000FD | GIGSR_1000HD)

/*******************MMD ACCESS  Control Register***************************/
#define MMD_CR_ADDR            (0x0000U)
#define MMD_CR_DATA_NOPOSTINC  (0x4000U)
#define MMD_CR_DATA_POSTINC_RW (0x8000U)
#define MMD_CR_DATA_POSTINC_W  (0xC000U)
#define MMD_CR_DEVADDR

/**************1K EXTENDED STATUS REGISTER*******************************/
#define GIGESR_1000FD ENETPHY_BIT(13)
#define GIGESR_1000HD ENETPHY_BIT(12)
#define GIGESR_1000   (GIGESR_1000FD | GIGESR_1000HD)

/**************************************************************************
                        API function Prototypes
***************************************************************************/
extern uint32_t dp83869_getid(uint32_t mdioBaseAddr, uint32_t PHY_Add);
extern bool dp83869_AutoNegotiate(uint32_t mdioBaseAddr, uint32_t phyAddr, uint16_t advVal);
extern void dp83869_Reset(uint32_t mdioBaseAddr, uint32_t phyAddr);
extern bool dp83869_LinkStatusGet(uint32_t mdioBaseAddr, uint32_t phyAddr, volatile uint32_t retries);
extern bool dp83869_PartnerAbilityGet(uint32_t mdioBaseAddr, uint32_t phyAddr, uint16_t *ptnerAblty);
extern void dp83869_EnableLoopback(uint32_t mdioBaseAddr, uint32_t phyAddr);
extern void dp83640DisableLoopback(uint32_t mdioBaseAddr, uint32_t phyAddr);
extern bool dp83869PartnerSpdGet(uint32_t mdioBaseAddr, uint32_t phyAddr, uint16_t *ptnerAblty);
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif
#endif /* DP83869_H_ */
