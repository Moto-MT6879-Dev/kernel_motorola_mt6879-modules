/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef _GL_MET_H
#define _GL_MET_H

#if CFG_SUPPORT_MET_LOG
/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                             F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */
void met_log_print_data(uint8_t *buffer, uint32_t size,
	uint32_t project_id, uint32_t chip_id);
int met_log_start(struct GLUE_INFO *prGlueInfo);
int met_log_stop(struct GLUE_INFO *prGlueInfo);
#endif /* CFG_SUPPORT_MET_LOG */

#endif /* _GL_MET_H */
