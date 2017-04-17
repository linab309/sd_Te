#ifndef INI_PARSER_H
#define INI_PARSER_H
#ifdef __cplusplus
extern "C" {
#endif
struct tag_value_list;
 
struct ini_parser {
    struct tag_value_list * keyvalues;
    int (*parse_file)(struct ini_parser *, const char * file);
    int (*parse_string)(struct ini_parser *, const char *text);
    char * (*value)(struct ini_parser *, const char * key);
    void (*set_value)(struct ini_parser *, const char * key, const char * value);
    void (*remove)(struct ini_parser *, const char *key);
    int (*save_to_file)(struct ini_parser *, const char * file);
};
 
struct ini_parser * new_ini_parser();
void delete_ini_parser(struct ini_parser *);
 
#ifdef __cplusplus
}
#endif
#endif // INI_PARSER_H

