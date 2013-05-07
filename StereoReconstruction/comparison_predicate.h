#pragma once

enum comparison_predicate_mode {
	LESS_THAN	 = 0x01,
	EQUAL		 = 0x02,
	GREATER_THAN = 0x04,

	LESS_THAN_OR_EQUAL	  = EQUAL | LESS_THAN,
	NOT_EQUAL			  = LESS_THAN | GREATER_THAN,
	GREATER_THAN_OR_EQUAL = GREATER_THAN | EQUAL
};

template<typename _Vt, int _mode>
class comparison_predicate
{
	_Vt _gauge;

	template <int _mode>
	bool compare(const _Vt &suspect) { throw std::exception("Bad comparison_predicate mode", _mode); }

	template<> bool compare< LESS_THAN             >(const _Vt &suspect) { return suspect <  _gauge; }
	template<> bool compare< LESS_THAN_OR_EQUAL    >(const _Vt &suspect) { return suspect <= _gauge; }
	template<> bool compare< EQUAL                 >(const _Vt &suspect) { return suspect == _gauge; }
	template<> bool compare< GREATER_THAN_OR_EQUAL >(const _Vt &suspect) { return suspect >= _gauge; }
	template<> bool compare< GREATER_THAN          >(const _Vt &suspect) { return suspect >  _gauge; }
	template<> bool compare< NOT_EQUAL             >(const _Vt &suspect) { return suspect != _gauge; }

public:
	comparison_predicate(_Vt  &gauge) : _gauge(gauge)	{ }
	comparison_predicate(_Vt &&gauge) : _gauge(std::forward<_Vt>(gauge)) { }

	comparison_predicate &operator=(_Vt &&gauge) { _gauge = std::move<_Vt>(gauge); }

	bool operator() (const _Vt &suspect) { return compare<_mode>(suspect); }
};