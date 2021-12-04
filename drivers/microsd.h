#ifndef _MICROSD_H_
#define _MICROSD_H_

typedef struct {
	char* filename;
	char* content;
} microsd_write_operation_t;

/**
 * @brief Function for writing content to microSD card.
 * @details If the file filename does not exist, it is created, and content is
 * written to this file. If the file already exists, content is appended to the
 * existing file.
 *
 * @param[in] filename	Name of file to write to.
 * @param[in] data		Data to write/append to file.
 */
void microsd_write(char* filename, char* data);
void microsd_task();
#endif /* _MICROSD_H_ */