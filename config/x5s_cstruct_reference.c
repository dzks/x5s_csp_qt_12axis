/* Master 0, Slave 0, "X5_7SEtherCAT"
 * Vendor ID:       0x00000766
 * Product code:    0x00010000
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x607a, 0x00, 32}, /* Target Position */
    {0x6060, 0x00, 16}, /* Modes of Operation */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position Actual Value */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1601, 3, slave_0_pdo_entries + 0}, /* csp RxPDO */
    {0x1a01, 2, slave_0_pdo_entries + 3}, /* csp TxPDO */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
