import time

class Byte_Var:
    """
    C-like byte类型变量与python泛型变量的转换类
    使用时直接操作成员bytes和value即可，已简化实现
    """
    def __init__(self, ctype="u8", var_type=int, value_multiplier=1, name=None):
        self._value = 0
        self._last_update_time = 0
        self.name = name
        self.reset(0, ctype, var_type, value_multiplier)

    def reset(self, init_value, ctype, py_var_type, value_multiplier=1, name=None):
        """重置变量"""
        ctype_word_part = ctype[0]
        ctype_number_part = ctype[1:]
        self._signed = (ctype_word_part.lower() == "s")
        self._byte_length = int(int(ctype_number_part) // 8)
        self._var_type = py_var_type
        self._multiplier = value_multiplier
        self._value = self._var_type(init_value)
        self._last_update_time = time.time()
        if name is not None:
            self.name = name
        return self

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = self._var_type(value)
        self._last_update_time = time.time()

    def update_value_with_mul(self, value):
        self._value = self._var_type(value * self._multiplier)

    @property
    def bytes(self):
        int_val = int(round(self._value / self._multiplier)) if self._multiplier != 1 else int(self._value)
        # 使用位置参数
        return int_val.to_bytes(self._byte_length, "little", self._signed)

    @bytes.setter
    def bytes(self, value):
        # 使用位置参数
        self._value = self._var_type(
            int.from_bytes(value, "little", self._signed) * self._multiplier
        )
        self._last_update_time = time.time()

    @property
    def byte_length(self):
        return self._byte_length

    @property
    def last_update_time(self):
        return self._last_update_time

    @property
    def struct_fmt_type(self):
        base_dict = {1: "b", 2: "h", 4: "i", 8: "q"}
        if self._signed:
            return base_dict[self._byte_length]
        else:
            return base_dict[self._byte_length].upper()