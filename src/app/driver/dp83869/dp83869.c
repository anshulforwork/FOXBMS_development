/*
 * dp83869.c
 *
 *  Created on: 29-May-2024
 *      Author: Aartech_Gourav_Sankadia
 */

#include "dp83869.h"

#include "HL_mdio.h"
#include "HL_sys_common.h"

//* \brief   Reads the PHY ID.
// *
// * \param   mdioBaseAddr  Base Address of the MDIO Module Registers.
// * \param   phyAddr       PHY Adress.
// *
// * \return  32 bit PHY ID (ID1:ID2)
// *
// **/
uint32_t dp83869_getid(uint32_t MdioBaseAdd, uint32_t PHY_Add) {
    uint32_t id   = 0;
    uint16_t data = 0;

    //Read IDR1 register
    (void)MDIOPhyRegRead(MdioBaseAdd, PHY_Add, (uint32_t)PHY_PHYIDR1, &data);

    id = (uint32_t)data;
    id = (uint32_t)((uint32_t)id << PHY_ID_SHIFT);
    //Read IDR2 register
    (void)MDIOPhyRegRead(MdioBaseAdd, PHY_Add, (uint32_t)PHY_PHYIDR2, &data);
    //update idr 2 value
    id |= data;
    return id;
}
void dp83869_Reset(uint32_t mdioBaseAddr, uint32_t phyAddr) {
    uint16_t regVal  = 0U;
    uint16_t *regPtr = &regVal;
    MDIOPhyRegWrite(mdioBaseAddr, phyAddr, PHY_BMCR, BMCR_RESET);

    (void)MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMCR, regPtr);
    /* : This bit is self-clearing and returns 1 until the reset process is complete. */
    while ((regVal & BMCR_RESET) != 0U) {
        (void)MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMCR, regPtr);
    }
}
bool dp83869_AutoNegotiate(uint32_t mdioBaseAddr, uint32_t phyAddr, uint16_t advVal) {
    volatile uint16_t data = 0U, anar = 0U;
    boolean retVal       = TRUE;
    uint32_t phyNegTries = 0xFFFFU;
    if (MDIOPhyRegRead(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMCR, &data) != TRUE) {
        retVal = FALSE;
    }

    data |= BMCR_ANEN;

    /* Enable Auto Negotiation */
    MDIOPhyRegWrite(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMCR, data);

    if (MDIOPhyRegRead(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMCR, &data) != TRUE) {
        retVal = FALSE;
    }

    /* Write Auto Negotiation capabilities */
    (void)MDIOPhyRegRead(mdioBaseAddr, phyAddr, (uint32_t)PHY_ANAR, &anar);
    anar &= (uint16_t)(~0xff10U);
    /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
    MDIOPhyRegWrite(mdioBaseAddr, phyAddr, (uint32_t)PHY_ANAR, (anar | advVal));

    data |= BMCR_ANRESTART;

    /* Start Auto Negotiation */
    MDIOPhyRegWrite(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMCR, data);

    /* Get the auto negotiation status*/
    if (MDIOPhyRegRead(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMSR, &data) != TRUE) {
        retVal = FALSE;
    }

    /* Wait till auto negotiation is complete */
    /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status bit read check" */
    while ((((uint16_t)(BMSR_ANINCOMPLETE)) == (data & (uint16_t)(BMSR_ANSTATUS))) && (retVal == TRUE) &&
           (phyNegTries > 0U)) {
        (void)MDIOPhyRegRead(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMSR, &data);
        phyNegTries--;
    }

    /* Check if the PHY is able to perform auto negotiation */
    /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
    if ((data & BMSR_ANCAPABLE) != 0U) {
        retVal = TRUE;
    } else {
        retVal = FALSE;
    }

    return retVal;
}

/**
 * \brief   Reads the link status of the PHY.
 *
 * \param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * \param   phyAddr       PHY Adress.
 * \param   retries       The number of retries before indicating down status
 *
 * \return  link status after reading \n
 *          TRUE if link is up
 *          FALSE if link is down \n
 *
 * \note    This reads both the basic status register of the PHY and the
 *          link register of MDIO for double check
 **/

bool dp83869_LinkStatusGet(uint32_t mdioBaseAddr, uint32_t phyAddr, volatile uint32_t retries) {
    volatile uint16_t linkStatus = 0U;
    boolean retVal               = TRUE;

    while (retVal == TRUE) {
        /* First read the BSR of the PHY */
        (void)MDIOPhyRegRead(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMSR, &linkStatus);

        /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
        if ((linkStatus & BMSR_LINKSTS) != 0U) {
            /* Check if MDIO LINK register is updated */
            linkStatus = (uint16_t)MDIOPhyLinkStatusGet(mdioBaseAddr);

            /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
            if ((linkStatus & (uint16_t)((uint16_t)1U << phyAddr)) != 0U) {
                break;
            } else {
                if (retries != 0U) {
                    retries--;
                } else {
                    retVal = FALSE;
                }
            }
        } else {
            if (retries != 0U) {
                retries--;
            } else {
                retVal = FALSE;
            }
        }
    }
    return retVal;
}

/**
 * \brief   Reads the Link Partner Ability register of the PHY.
 *
 * \param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * \param   phyAddr       PHY Adress.
 * \param   ptnerAblty    The partner abilities of the EMAC
 *
 * \return  status after reading \n
 *          TRUE if reading successful
 *          FALSE if reading failed
 **/

bool dp83869_PartnerAbilityGet(uint32_t mdioBaseAddr, uint32_t phyAddr, uint16_t *ptnerAblty) {
    return (MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_ANLPAR, ptnerAblty));
}

/**
 * \brief   Enables PHY Loopback.
 *
 * \param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * \param   phyAddr       PHY Adress.
 *
 * \return  No return value.
 **/
void dp83869_EnableLoopback(uint32_t mdioBaseAddr, uint32_t phyAddr) {
    uint32_t delay  = 0x1FFFU;
    uint16_t regVal = 0x0000U;
    uint16 *regPtr  = &regVal;
    (void)MDIOPhyRegRead(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMCR, regPtr);
    /* Disabling Auto Negotiate. */
    /*SAFETYMCUSW 334 S MR:10.5 <APPROVED> "Only unsigned short values are used." */
    regVal &= (uint16_t)(~((uint16_t)BMCR_ANEN));
    /* Enabling Loopback. */
    regVal |= BMCR_LOOPBACK;

    MDIOPhyRegWrite(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMCR, regVal);

    while (delay > 0U) {
        delay--;
    }
}

/**
 * \brief   Disable PHY Loopback.
 *
 * \param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * \param   phyAddr       PHY Adress.
 *
 * \return  No return value.
 **/

void Dp83640DisableLoopback(uint32_t mdioBaseAddr, uint32_t phyAddr) {
    uint32_t delay   = 0x1FFFU;
    uint16_t regVal  = 0x0000U;
    uint16_t *regPtr = &regVal;
    (void)MDIOPhyRegRead(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMCR, regPtr);

    /* Enabling Loopback. */
    /*SAFETYMCUSW 334 S MR:10.5 <APPROVED> "Only unsigned short values are used." */
    regVal &= (uint16_t)(~((uint16_t)BMCR_ANEN));

    MDIOPhyRegWrite(mdioBaseAddr, phyAddr, (uint32_t)PHY_BMCR, regVal);

    while (delay > 0U) {
        delay--;
    }
}

/**
 * \brief   Reads the Speed info from Status register of the PHY.
 *
 * \param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * \param   phyAddr       PHY Adress.
 * \param   ptnerAblty    The partner abilities of the EMAC
 *
 * \return  status after reading \n
 *          TRUE if reading successful
 *          FALSE if reading failed
 **/
bool Dp83869PartnerSpdGet(uint32_t mdioBaseAddr, uint32_t phyAddr, uint16_t *ptnerAblty) {
    return (MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_STATUS, ptnerAblty));
}
