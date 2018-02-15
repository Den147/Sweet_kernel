/*
 * Copyright (C) 2017-2018, Alex Saiko <solcmdr@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __WCD9XXX_SND_CTRL_H__
#define __WCD9XXX_SND_CTRL_H__

#include <linux/list.h>
#include <sound/soc.h>

/*
 * Ignore write attemptions coming from IOCTL to handled registers.
 * Note that sound codec must utilize this flag in its filter call,
 * otherwise it won't change anything.
 */
#define SND_CTRL_BYPASS_IOCTL	BIT(0)

struct snd_ctrl_lines {
	u32 mic_line;
	u32 cam_mic_line;
	u32 speaker_l_line;
	u32 speaker_r_line;
	u32 headphone_l_line;
	u32 headphone_r_line;
};

struct snd_ctrl_data {
	/* Sound codec conjuncted to a control data */
	struct snd_soc_codec *codec;
	struct list_head member;

	/* Control data's name */
	const char *name;

	/* Basic audio input lines */
	struct snd_ctrl_lines lines;

	/* Data-specific control flags */
	unsigned long flags;

	/* Codec's I/O functions used to access to sound registers */
	unsigned int	(*read)		(struct snd_soc_codec *codec,
					 unsigned int reg);
	int		(*write)	(struct snd_soc_codec *codec,
					 unsigned int reg, unsigned int val);
};

int snd_ctrl_register(struct snd_ctrl_data *snd_data);
void snd_ctrl_unregister(struct snd_ctrl_data *snd_data);
bool snd_ctrl_data_handled(struct snd_ctrl_data *snd_data);

#endif /* __WCD9XXX_SND_CTRL_H__ */
